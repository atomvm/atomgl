/*
 * This file is part of AtomGL.
 *
 * Copyright 2026 Davide Bettio <davide@uninstall.it>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "display_diff.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "display_damage.h"
#include "display_items.h"
#include "font_data.h"
#include "shape.h"

#define SCREEN_W 48
#define SCREEN_H 32
#define MAX_ITEMS 16
#define POOL_IMG_SIZE 6
#define POOL_LEN 6

static int failures = 0;

#define CHECK(cond)                                                                  \
    do {                                                                             \
        if (!(cond)) {                                                               \
            fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #cond); \
            failures++;                                                              \
        }                                                                            \
    } while (0)

static bool damage_covers_pixel(const struct DisplayDamage *damage, int x, int y)
{
    for (size_t i = 0; i < damage->rects_len; i++) {
        const struct DisplayRect *r = &damage->rects[i];
        if (x >= r->x && x < r->x + r->width && y >= r->y && y < r->y + r->height) {
            return true;
        }
    }

    return false;
}

static bool damage_covers_rect(const struct DisplayDamage *damage, int x, int y, int w, int h)
{
    for (int py = y; py < y + h; py++) {
        for (int px = x; px < x + w; px++) {
            if (px < 0 || px >= damage->screen_width || py < 0 || py >= damage->screen_height) {
                continue;
            }
            if (!damage_covers_pixel(damage, px, py)) {
                return false;
            }
        }
    }

    return true;
}

static long damage_area(const struct DisplayDamage *damage)
{
    long area = 0;
    for (int y = 0; y < damage->screen_height; y++) {
        for (int x = 0; x < damage->screen_width; x++) {
            area += damage_covers_pixel(damage, x, y) ? 1 : 0;
        }
    }

    return area;
}

// --- item constructors ---

static BaseDisplayItem make_rect(int x, int y, int w, int h, uint32_t rgb)
{
    BaseDisplayItem item;
    memset(&item, 0, sizeof(item));
    item.primitive = PrimitiveRect;
    item.x = x;
    item.y = y;
    item.width = w;
    item.height = h;
    item.brcolor = rgb << 8 | 0xFF;

    return item;
}

static BaseDisplayItem make_text(int x, int y, uint32_t fg, uint32_t bg, const char *text)
{
    BaseDisplayItem item;
    memset(&item, 0, sizeof(item));
    item.primitive = PrimitiveText;
    item.x = x;
    item.y = y;
    item.width = strlen(text) * CHAR_WIDTH;
    item.height = 16;
    item.brcolor = bg;
    item.data.text_data.fgcolor = fg << 8 | 0xFF;
    item.data.text_data.text = text;

    return item;
}

static BaseDisplayItem make_image(int x, int y, uint32_t bg, int w, int h, const char *pix)
{
    BaseDisplayItem item;
    memset(&item, 0, sizeof(item));
    item.primitive = PrimitiveImage;
    item.x = x;
    item.y = y;
    item.width = w;
    item.height = h;
    item.brcolor = bg;
    item.data.image_data.pix = pix;

    return item;
}

static BaseDisplayItem make_scaled(int x, int y, int w, int h, uint32_t bg, int source_x,
    int source_y, int x_scale, int y_scale, int img_w, int img_h, const char *pix)
{
    BaseDisplayItem item;
    memset(&item, 0, sizeof(item));
    item.primitive = PrimitiveScaledCroppedImage;
    item.x = x;
    item.y = y;
    item.width = w;
    item.height = h;
    item.brcolor = bg;
    item.source_x = source_x;
    item.source_y = source_y;
    item.x_scale = x_scale;
    item.y_scale = y_scale;
    item.data.image_data_with_size.width = img_w;
    item.data.image_data_with_size.height = img_h;
    item.data.image_data_with_size.pix = pix;

    return item;
}

static BaseDisplayItem make_shape(struct ShapeData *shape, uint32_t rgb)
{
    BaseDisplayItem item;
    memset(&item, 0, sizeof(item));
    item.primitive = PrimitiveShape;
    item.brcolor = rgb << 8 | 0xFF;
    item.data.shape_data.shape = shape;
    shape_bounds(shape, &item.x, &item.y, &item.width, &item.height);

    return item;
}

static void compute(const BaseDisplayItem *old_items, size_t old_len,
    const BaseDisplayItem *new_items, size_t new_len, struct DisplayDamage *damage)
{
    display_damage_init(damage, SCREEN_W, SCREEN_H);
    display_diff_compute(old_items, old_len, new_items, new_len, damage);
}

// --- reference renderer ---
//
// Must match the per-pixel semantics of the scanline renderers: the topmost
// item covering the pixel that is opaque there decides the color. Blended
// pixels (translucent image pixel over the item's own background) are encoded
// as a distinct value derived from both colors.

#define TRANSPARENT UINT64_MAX

static uint32_t read_rgba(const char *pix, size_t index)
{
    const uint8_t *p = (const uint8_t *) pix + index * 4;

    return ((uint32_t) p[0] << 24) | ((uint32_t) p[1] << 16) | ((uint32_t) p[2] << 8) | p[3];
}

static uint64_t image_pixel_color(uint32_t rgba, uint32_t brcolor)
{
    uint8_t alpha = rgba & 0xFF;
    if (alpha == 0xFF) {
        return rgba;
    }
    if (brcolor != 0) {
        return ((uint64_t) 1 << 40) | ((uint64_t) rgba << 8) | (brcolor >> 8) | ((uint64_t) alpha << 48);
    }

    return TRANSPARENT;
}

// The documented mapping: with a flip the axis is mirrored within the part of the item the
// image covers, pixels past it show the edge pixel.
static int scaled_source(int px, int len, int src_len, int scale, bool flip)
{
    if (flip) {
        int covered = src_len * scale < len ? src_len * scale : len;
        px = covered - 1 - px;
        if (px < 0) {
            px = 0;
        }
    }
    int src = px / scale;

    return src < src_len - 1 ? src : src_len - 1;
}

static uint64_t item_color_at(const BaseDisplayItem *item, int x, int y)
{
    int dx = x - item->x;
    int dy = y - item->y;

    switch (item->primitive) {
        case PrimitiveRect:
            return item->brcolor;

        case PrimitiveText: {
            char c = item->data.text_data.text[dx / CHAR_WIDTH];
            const unsigned char *glyph = fontdata + ((unsigned char) c) * 16;
            unsigned char row = glyph[dy];
            if (row & (1 << (7 - (dx % CHAR_WIDTH)))) {
                return item->data.text_data.fgcolor;
            }
            return item->brcolor != 0 ? item->brcolor : TRANSPARENT;
        }

        case PrimitiveImage: {
            uint32_t rgba = read_rgba(item->data.image_data.pix, (size_t) dy * item->width + dx);
            return image_pixel_color(rgba, item->brcolor);
        }

        case PrimitiveScaledCroppedImage: {
            const struct ImageDataWithSize *img = &item->data.image_data_with_size;
            int sx = item->source_x + scaled_source(dx, item->width, img->width - item->source_x, item->x_scale, item->flip_x);
            int sy = item->source_y + scaled_source(dy, item->height, img->height - item->source_y, item->y_scale, item->flip_y);
            uint32_t rgba = read_rgba(img->pix, (size_t) sy * img->width + sx);
            return image_pixel_color(rgba, item->brcolor);
        }

        case PrimitiveShape:
            return shape_contains(item->data.shape_data.shape, x, y) ? item->brcolor : TRANSPARENT;

        default:
            return TRANSPARENT;
    }
}

static bool item_covers(const BaseDisplayItem *item, int x, int y)
{
    return x >= item->x && x < item->x + item->width && y >= item->y && y < item->y + item->height;
}

// Black background: uncovered pixels are cleared by the drivers.
static uint64_t ref_color(const BaseDisplayItem items[], size_t items_len, int x, int y)
{
    for (size_t i = 0; i < items_len; i++) {
        if (!item_covers(&items[i], x, y)) {
            continue;
        }
        uint64_t color = item_color_at(&items[i], x, y);
        if (color != TRANSPARENT) {
            return color;
        }
    }

    return 0x000000FF;
}

// What the diff guarantees outside the damage; stricter than equal colors.
static bool same_covering_sequence(const BaseDisplayItem old_items[], size_t old_len,
    const BaseDisplayItem new_items[], size_t new_len, int x, int y)
{
    size_t i = 0;
    size_t j = 0;
    while (true) {
        while (i < old_len && !item_covers(&old_items[i], x, y)) {
            i++;
        }
        while (j < new_len && !item_covers(&new_items[j], x, y)) {
            j++;
        }
        if (i == old_len || j == new_len) {
            return i == old_len && j == new_len;
        }
        if (!display_diff_items_equal(&old_items[i], &new_items[j])) {
            return false;
        }
        i++;
        j++;
    }
}

// --- unit tests ---

static const char *const texts[] = { "", "A", "AB", "BA", "ABC" };
static char pool[POOL_LEN][POOL_IMG_SIZE * POOL_IMG_SIZE * 4];

static void init_pool(void)
{
    for (int i = 0; i < POOL_LEN; i++) {
        for (int j = 0; j < POOL_IMG_SIZE * POOL_IMG_SIZE; j++) {
            uint8_t *p = (uint8_t *) &pool[i][j * 4];
            p[0] = rand() & 0xFF;
            p[1] = rand() & 0xFF;
            p[2] = rand() & 0xFF;
            int alpha_kind = rand() % 3;
            p[3] = alpha_kind == 0 ? 0 : (alpha_kind == 1 ? 0x80 : 0xFF);
        }
    }
}

static void test_items_equal(void)
{
    BaseDisplayItem a = make_rect(1, 2, 3, 4, 0x112233);
    BaseDisplayItem b = a;
    CHECK(display_diff_items_equal(&a, &b));
    b.x++;
    CHECK(!display_diff_items_equal(&a, &b));
    b = a;
    b.height++;
    CHECK(!display_diff_items_equal(&a, &b));
    b = make_rect(1, 2, 3, 4, 0x112234);
    CHECK(!display_diff_items_equal(&a, &b));

    char text_copy[] = "AB";
    BaseDisplayItem t1 = make_text(0, 0, 0xFF0000, 0, "AB");
    BaseDisplayItem t2 = make_text(0, 0, 0xFF0000, 0, text_copy);
    CHECK(display_diff_items_equal(&t1, &t2));
    t2 = make_text(0, 0, 0xFF0001, 0, "AB");
    CHECK(!display_diff_items_equal(&t1, &t2));
    t2 = make_text(0, 0, 0xFF0000, 0xFF, "AB");
    CHECK(!display_diff_items_equal(&t1, &t2));
    t2 = make_text(0, 0, 0xFF0000, 0, "BA");
    CHECK(!display_diff_items_equal(&t1, &t2));
    t2 = make_text(0, 0, 0xFF0000, 0, "ABC");
    CHECK(!display_diff_items_equal(&t1, &t2));
    CHECK(!display_diff_items_equal(&a, &t1));

    char copy[sizeof(pool[0])];
    memcpy(copy, pool[0], sizeof(copy));
    BaseDisplayItem i1 = make_image(3, 3, 0, 6, 6, pool[0]);
    BaseDisplayItem i2 = make_image(3, 3, 0, 6, 6, pool[0]);
    CHECK(display_diff_items_equal(&i1, &i2));
    // Same pixels in another buffer: the content is never compared.
    i2 = make_image(3, 3, 0, 6, 6, copy);
    CHECK(!display_diff_items_equal(&i1, &i2));
    i2 = make_image(3, 3, 0, 6, 5, pool[0]);
    CHECK(!display_diff_items_equal(&i1, &i2));
    i2 = make_image(3, 3, 0xFF, 6, 6, pool[0]);
    CHECK(!display_diff_items_equal(&i1, &i2));
    i2 = make_image(3, 3, 0, 0, 0, NULL);
    BaseDisplayItem i3 = make_image(3, 3, 0, 0, 0, NULL);
    CHECK(display_diff_items_equal(&i2, &i3));
    i3 = make_image(3, 3, 0, 0, 0, pool[1]);
    CHECK(!display_diff_items_equal(&i2, &i3));

    BaseDisplayItem s1 = make_scaled(0, 0, 8, 8, 0, 1, 1, 2, 2, 6, 6, pool[2]);
    BaseDisplayItem s2 = s1;
    CHECK(display_diff_items_equal(&s1, &s2));
    memcpy(copy, pool[2], sizeof(copy));
    s2 = make_scaled(0, 0, 8, 8, 0, 1, 1, 2, 2, 6, 6, copy);
    CHECK(!display_diff_items_equal(&s1, &s2));
    s2 = s1;
    s2.source_x = 2;
    CHECK(!display_diff_items_equal(&s1, &s2));
    s2 = s1;
    s2.y_scale = 1;
    CHECK(!display_diff_items_equal(&s1, &s2));
    s2 = s1;
    s2.data.image_data_with_size.width = 5;
    CHECK(!display_diff_items_equal(&s1, &s2));
    s2 = s1;
    s2.data.image_data_with_size.pix = pool[3];
    CHECK(!display_diff_items_equal(&s1, &s2));

    BaseDisplayItem inv1;
    BaseDisplayItem inv2;
    memset(&inv1, 0, sizeof(inv1));
    memset(&inv2, 0, sizeof(inv2));
    CHECK(display_diff_items_equal(&inv1, &inv2));
    CHECK(!display_diff_items_equal(&inv1, &a));

    s2 = s1;
    s2.flip_x = true;
    CHECK(!display_diff_items_equal(&s1, &s2));
    s2 = s1;
    s2.flip_y = true;
    CHECK(!display_diff_items_equal(&s1, &s2));

    // Shapes compare by geometry, not by object: a re-parsed identical shape is unchanged.
    struct ShapeData *c1 = shape_new_ellipse(10, 10, 5, 5);
    struct ShapeData *c2 = shape_new_ellipse(10, 10, 5, 5);
    struct ShapeData *c3 = shape_new_ellipse(10, 10, 5, 4);
    struct ShapePoint tri[] = { { 0, 0 }, { 10, 0 }, { 0, 10 } };
    struct ShapeData *p1 = shape_new_polygon(tri, 3);
    BaseDisplayItem sh1 = make_shape(c1, 0x00FF00);
    BaseDisplayItem sh2 = make_shape(c2, 0x00FF00);
    CHECK(display_diff_items_equal(&sh1, &sh2));
    sh2 = make_shape(c3, 0x00FF00);
    CHECK(!display_diff_items_equal(&sh1, &sh2));
    sh2 = make_shape(c2, 0x00FF01);
    CHECK(!display_diff_items_equal(&sh1, &sh2));
    sh2 = make_shape(p1, 0x00FF00);
    CHECK(!display_diff_items_equal(&sh1, &sh2));
    CHECK(!display_diff_items_equal(&sh1, &a));
    shape_destroy(c1);
    shape_destroy(c2);
    shape_destroy(c3);
    shape_destroy(p1);
}

static void test_diff_scenarios(void)
{
    struct DisplayDamage damage;
    BaseDisplayItem bg = make_rect(0, 0, SCREEN_W, SCREEN_H, 0x808080);
    BaseDisplayItem label = make_text(4, 4, 0x000000, 0, "AB");
    BaseDisplayItem label2 = make_text(4, 4, 0x000000, 0, "BA");
    BaseDisplayItem icon = make_image(30, 10, 0, 6, 6, pool[0]);
    BaseDisplayItem icon_moved = make_image(20, 20, 0, 6, 6, pool[0]);

    // Identical lists: nothing to repaint.
    BaseDisplayItem list1[] = { label, icon, bg };
    compute(list1, 3, list1, 3, &damage);
    CHECK(display_damage_is_empty(&damage));
    compute(NULL, 0, NULL, 0, &damage);
    CHECK(display_damage_is_empty(&damage));

    // First frame: every item damaged.
    compute(NULL, 0, list1, 3, &damage);
    CHECK(display_damage_is_full(&damage));

    // Everything removed: every old item damaged.
    compute(list1, 3, NULL, 0, &damage);
    CHECK(display_damage_is_full(&damage));

    // In-place text change: only the label area.
    BaseDisplayItem list2[] = { label2, icon, bg };
    compute(list1, 3, list2, 3, &damage);
    CHECK(damage.rects_len == 1);
    CHECK(damage_covers_rect(&damage, 4, 4, 16, 16));
    CHECK(damage_area(&damage) == 16 * 16);

    // Insert at head, in the middle and at the tail (tail insert is under the
    // opaque background but the diff does not know that: bbox damaged).
    BaseDisplayItem popup = make_rect(10, 10, 5, 5, 0xFF0000);
    BaseDisplayItem list3[] = { popup, label, icon, bg };
    compute(list1, 3, list3, 4, &damage);
    CHECK(damage_area(&damage) == 25);
    CHECK(damage_covers_rect(&damage, 10, 10, 5, 5));
    BaseDisplayItem list4[] = { label, popup, icon, bg };
    compute(list1, 3, list4, 4, &damage);
    CHECK(damage_area(&damage) == 25);
    BaseDisplayItem list5[] = { label, icon, bg, popup };
    compute(list1, 3, list5, 4, &damage);
    CHECK(damage_area(&damage) == 25);

    // Removal: only the removed item's area.
    compute(list4, 4, list1, 3, &damage);
    CHECK(damage_area(&damage) == 25);
    BaseDisplayItem list6[] = { label, bg };
    compute(list1, 3, list6, 2, &damage);
    CHECK(damage_area(&damage) == 36);
    CHECK(damage_covers_rect(&damage, 30, 10, 6, 6));

    // Moved item: old and new position.
    BaseDisplayItem list7[] = { label, icon_moved, bg };
    compute(list1, 3, list7, 3, &damage);
    CHECK(damage_area(&damage) == 72);
    CHECK(damage_covers_rect(&damage, 30, 10, 6, 6));
    CHECK(damage_covers_rect(&damage, 20, 20, 6, 6));

    // Swapped overlapping items: at least one of them is repainted.
    BaseDisplayItem r1 = make_rect(0, 0, 10, 10, 0xFF0000);
    BaseDisplayItem r2 = make_rect(5, 5, 10, 10, 0x00FF00);
    BaseDisplayItem list8[] = { r1, r2 };
    BaseDisplayItem list9[] = { r2, r1 };
    compute(list8, 2, list9, 2, &damage);
    CHECK(damage_covers_rect(&damage, 5, 5, 5, 5));
    CHECK(damage_covers_rect(&damage, 0, 0, 10, 10) || damage_covers_rect(&damage, 5, 5, 10, 10));

    // Duplicates.
    BaseDisplayItem list10[] = { r1, r1 };
    BaseDisplayItem list11[] = { r1 };
    compute(list10, 2, list11, 1, &damage);
    CHECK(damage_area(&damage) == 100);
    compute(list11, 1, list10, 2, &damage);
    CHECK(damage_area(&damage) == 100);

    // Same image content through a different binary: repainted, the pixels
    // are never compared.
    char copy[sizeof(pool[0])];
    memcpy(copy, pool[0], sizeof(copy));
    BaseDisplayItem icon_copy = make_image(30, 10, 0, 6, 6, copy);
    BaseDisplayItem list12[] = { label, icon_copy, bg };
    compute(list1, 3, list12, 3, &damage);
    CHECK(damage_area(&damage) == 36);
    CHECK(damage_covers_rect(&damage, 30, 10, 6, 6));

    // Off screen and invalid items never produce damage.
    BaseDisplayItem off = make_rect(-20, -20, 10, 10, 0xFF0000);
    BaseDisplayItem inv;
    memset(&inv, 0, sizeof(inv));
    BaseDisplayItem list13[] = { off, inv, label, icon, bg };
    compute(list1, 3, list13, 5, &damage);
    CHECK(display_damage_is_empty(&damage));
}

// --- randomized property test ---

struct Owned
{
    char *ptrs[64];
    size_t len;
    struct ShapeData *shapes[64];
    size_t shapes_len;
};

static struct ShapeData *owned_shape(struct Owned *owned, struct ShapeData *shape)
{
    if (shape != NULL && owned->shapes_len < 64) {
        owned->shapes[owned->shapes_len++] = shape;
    }

    return shape;
}

static char *owned_copy(struct Owned *owned, const char *src, size_t size)
{
    char *copy = malloc(size);
    memcpy(copy, src, size);
    if (owned->len < 64) {
        owned->ptrs[owned->len++] = copy;
    }

    return copy;
}

static void owned_free(struct Owned *owned)
{
    for (size_t i = 0; i < owned->len; i++) {
        free(owned->ptrs[i]);
    }
    owned->len = 0;
    for (size_t i = 0; i < owned->shapes_len; i++) {
        shape_destroy(owned->shapes[i]);
    }
    owned->shapes_len = 0;
}

static struct ShapeData *random_shape(struct Owned *owned)
{
    int x = rand() % (SCREEN_W + 16) - 8;
    int y = rand() % (SCREEN_H + 16) - 8;
    struct ShapeData *shape = NULL;
    switch (rand() % 5) {
        case 0:
            shape = shape_new_rounded_rect(x, y, 1 + rand() % 20, 1 + rand() % 16, rand() % 8);
            break;
        case 1:
            shape = shape_new_ellipse(x, y, 1 + rand() % 10, 1 + rand() % 8);
            break;
        case 2:
            shape = shape_new_line(x, y, rand() % SCREEN_W, rand() % SCREEN_H, 1 + rand() % 5);
            break;
        case 3: {
            int radius = 1 + rand() % 10;
            shape = shape_new_arc(x, y, radius, 1 + rand() % (radius + 2), rand() % 360, rand() % 360);
            break;
        }
        default: {
            struct ShapePoint points[6];
            int n = 3 + rand() % 4;
            for (int i = 0; i < n; i++) {
                points[i].x = x + rand() % 20;
                points[i].y = y + rand() % 16;
            }
            shape = shape_new_polygon(points, n);
            break;
        }
    }

    return owned_shape(owned, shape);
}

static const uint32_t palette[] = { 0x000000, 0xFFFFFF, 0xFF0000, 0x0080FF };

static uint32_t random_bg(void)
{
    int k = rand() % 3;
    if (k == 0) {
        return 0;
    }

    return palette[rand() % 4] << 8 | 0xFF;
}

static BaseDisplayItem random_item(struct Owned *owned)
{
    int x = rand() % (SCREEN_W + 16) - 8;
    int y = rand() % (SCREEN_H + 16) - 8;
    int kind = rand() % 20;

    if (kind < 7) {
        return make_rect(x, y, rand() % 24, rand() % 20, palette[rand() % 4]);
    }
    if (kind < 11) {
        return make_text(x, y, palette[rand() % 2], random_bg(), texts[rand() % 5]);
    }
    if (kind < 14) {
        int w = 1 + rand() % POOL_IMG_SIZE;
        int h = 1 + rand() % POOL_IMG_SIZE;
        const char *pix = pool[rand() % POOL_LEN];
        if (rand() % 3 == 0) {
            pix = owned_copy(owned, pix, sizeof(pool[0]));
        }
        return make_image(x, y, random_bg(), w, h, pix);
    }
    if (kind < 17) {
        int source_x = rand() % POOL_IMG_SIZE;
        int source_y = rand() % POOL_IMG_SIZE;
        int x_scale = 1 + rand() % 3;
        int y_scale = 1 + rand() % 3;
        int max_w = (POOL_IMG_SIZE - source_x) * x_scale;
        int max_h = (POOL_IMG_SIZE - source_y) * y_scale;
        int w = rand() % (max_w + 1);
        int h = rand() % (max_h + 1);
        const char *pix = pool[rand() % POOL_LEN];
        if (rand() % 3 == 0) {
            pix = owned_copy(owned, pix, sizeof(pool[0]));
        }
        BaseDisplayItem item = make_scaled(x, y, w, h, random_bg(), source_x, source_y, x_scale,
            y_scale, POOL_IMG_SIZE, POOL_IMG_SIZE, pix);
        item.flip_x = rand() % 2;
        item.flip_y = rand() % 2;
        return item;
    }
    if (kind < 19) {
        struct ShapeData *shape = random_shape(owned);
        if (shape != NULL) {
            return make_shape(shape, palette[rand() % 4]);
        }
        return make_rect(x, y, rand() % 24, rand() % 20, palette[rand() % 4]);
    }

    BaseDisplayItem invalid;
    memset(&invalid, 0, sizeof(invalid));

    return invalid;
}

static size_t random_list(BaseDisplayItem items[], size_t max_len, struct Owned *owned)
{
    size_t len = rand() % (max_len + 1);
    for (size_t i = 0; i < len; i++) {
        items[i] = random_item(owned);
    }

    return len;
}

// Returns false when the new list must not produce any damage.
static bool mutate(const BaseDisplayItem old_items[], size_t old_len, BaseDisplayItem new_items[],
    size_t *new_len, struct Owned *owned)
{
    memcpy(new_items, old_items, old_len * sizeof(BaseDisplayItem));
    size_t len = old_len;
    bool changed = false;

    int ops = rand() % 4;
    for (int op = 0; op < ops; op++) {
        int kind = rand() % 7;
        if (kind == 0 && len > 0) {
            new_items[rand() % len] = random_item(owned);
            changed = true;
        } else if (kind == 1 && len < MAX_ITEMS) {
            size_t at = rand() % (len + 1);
            memmove(&new_items[at + 1], &new_items[at], (len - at) * sizeof(BaseDisplayItem));
            new_items[at] = random_item(owned);
            len++;
            changed = true;
        } else if (kind == 2 && len > 0) {
            size_t at = rand() % len;
            memmove(&new_items[at], &new_items[at + 1], (len - at - 1) * sizeof(BaseDisplayItem));
            len--;
            changed = true;
        } else if (kind == 3 && len > 1) {
            size_t a = rand() % len;
            size_t b = rand() % len;
            BaseDisplayItem tmp = new_items[a];
            new_items[a] = new_items[b];
            new_items[b] = tmp;
            changed = changed || a != b;
        } else if (kind == 4 && len > 0 && len < MAX_ITEMS) {
            size_t at = rand() % len;
            memmove(&new_items[at + 1], &new_items[at], (len - at) * sizeof(BaseDisplayItem));
            len++;
            changed = true;
        } else if (kind == 5 && len > 0) {
            // Same pixels through another buffer: counts as a change.
            size_t at = rand() % len;
            BaseDisplayItem *item = &new_items[at];
            if (item->primitive == PrimitiveImage && item->data.image_data.pix != NULL) {
                item->data.image_data.pix = owned_copy(owned, item->data.image_data.pix, sizeof(pool[0]));
                changed = true;
            } else if (item->primitive == PrimitiveScaledCroppedImage) {
                item->data.image_data_with_size.pix
                    = owned_copy(owned, item->data.image_data_with_size.pix, sizeof(pool[0]));
                changed = true;
            }
        } else if (kind == 6 && len > 0) {
            // A shape's box must stay consistent with its geometry, so only other items move.
            size_t at = rand() % len;
            if (new_items[at].primitive == PrimitiveShape) {
                continue;
            }
            new_items[at].x += rand() % 5 - 2;
            new_items[at].y += rand() % 5 - 2;
            changed = true;
        }
    }

    *new_len = len;

    return changed;
}

static void test_random_property(void)
{
    static BaseDisplayItem old_items[MAX_ITEMS];
    static BaseDisplayItem new_items[MAX_ITEMS];
    struct Owned owned = { .len = 0, .shapes_len = 0 };

    for (int iteration = 0; iteration < 4000; iteration++) {
        size_t old_len = random_list(old_items, 12, &owned);
        size_t new_len;
        bool may_change;
        if (rand() % 4 == 0) {
            new_len = random_list(new_items, 12, &owned);
            may_change = true;
        } else {
            may_change = mutate(old_items, old_len, new_items, &new_len, &owned);
        }

        struct DisplayDamage damage;
        compute(old_items, old_len, new_items, new_len, &damage);

        if (!may_change) {
            CHECK(display_damage_is_empty(&damage));
        }

        for (int y = 0; y < SCREEN_H; y++) {
            for (int x = 0; x < SCREEN_W; x++) {
                if (damage_covers_pixel(&damage, x, y)) {
                    continue;
                }
                bool same_seq = same_covering_sequence(old_items, old_len, new_items, new_len, x, y);
                bool same_col = ref_color(old_items, old_len, x, y) == ref_color(new_items, new_len, x, y);
                CHECK(same_seq);
                CHECK(same_col);
                if (!same_seq || !same_col) {
                    fprintf(stderr, "iteration %i: pixel (%i, %i) outside damage changed\n",
                        iteration, x, y);
                    owned_free(&owned);
                    return;
                }
            }
        }

        owned_free(&owned);
    }
}

int main(void)
{
    srand(20260928);
    init_pool();

    test_items_equal();
    test_diff_scenarios();
    test_random_property();

    if (failures) {
        fprintf(stderr, "%i check(s) failed\n", failures);
        return EXIT_FAILURE;
    }
    printf("display_diff: all tests passed\n");

    return EXIT_SUCCESS;
}
