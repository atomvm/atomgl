/*
 * This file is part of AtomGL.
 *
 * Copyright 2026 Tom Hoenderdos <tomhoenderdos@gmail.com>
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

#include <limits.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <context.h>
#include <defaultatoms.h>
#include <globalcontext.h>
#include <memory.h>
#include <term.h>

#include "dcs_lcd_draw.h"
#include "dcs_lcd_screen.h"
#include "display_items.h"

#include "alloc_counter.h"

extern int term_display_non_atoms;

#define SCREEN_W 64
#define SCREEN_H 48
#define HEAP_TERMS (1 << 16)
#define MAX_BUFFERS 256

static term heap_buf[HEAP_TERMS];
static Heap heap;
static GlobalContext glb;
static Context ctx;

static void *buffers[MAX_BUFFERS];
static int buffers_len;

static int failures;
static int checks;

#define CHECK(cond, ...)                                         \
    do {                                                         \
        checks++;                                                \
        if (!(cond)) {                                           \
            failures++;                                          \
            fprintf(stderr, "FAIL %s:%d: ", __FILE__, __LINE__); \
            fprintf(stderr, __VA_ARGS__);                        \
            fprintf(stderr, "\n");                               \
        }                                                        \
    } while (0)

static term atom(const char *name)
{
    char buf[256];
    size_t len = strlen(name);
    buf[0] = (char) len;
    memcpy(buf + 1, name, len);
    return globalcontext_make_atom(&glb, (AtomString) buf);
}

static Heap *heap_with_room(size_t n)
{
    if (heap.heap_ptr + n > heap.heap_end) {
        fprintf(stderr, "test heap exhausted\n");
        abort();
    }
    return &heap;
}

static term boxed_int(avm_int64_t v)
{
    return term_make_boxed_int64(v, heap_with_room(1 + BOXED_TERMS_REQUIRED_FOR_INT64));
}

static term tuple(int n, ...)
{
    term t = term_alloc_tuple(n, heap_with_room(1 + n));
    va_list ap;
    va_start(ap, n);
    for (int i = 0; i < n; i++) {
        term_put_tuple_element(t, i, va_arg(ap, term));
    }
    va_end(ap);
    return t;
}

static term cons(term head, term tail)
{
    return term_list_init_prepend(memory_heap_alloc(heap_with_room(2), 2), head, tail);
}

static term list(int n, ...)
{
    term elems[64];
    va_list ap;
    va_start(ap, n);
    for (int i = 0; i < n; i++) {
        elems[i] = va_arg(ap, term);
    }
    va_end(ap);
    term l = term_nil();
    for (int i = n - 1; i >= 0; i--) {
        l = cons(elems[i], l);
    }
    return l;
}

static term pixels(size_t size)
{
    if (buffers_len == MAX_BUFFERS) {
        abort();
    }
    uint8_t *data = malloc(size ? size : 1);
    if (data == NULL) {
        abort();
    }
    for (size_t i = 0; i < size; i++) {
        size_t px = i / 4;
        switch (i % 4) {
            case 0:
                data[i] = (uint8_t) (((px * 13 + 17) % 32) << 3 | 5);
                break;
            case 1:
                data[i] = (uint8_t) ((((px / 32) * 21 + px * 3) % 64) << 2 | 2);
                break;
            case 2:
                data[i] = (uint8_t) (0xFF - 8 * (px % 32));
                break;
            default:
                data[i] = 0xFF;
        }
    }
    buffers[buffers_len++] = data;
    return term_from_const_binary(data, size, &heap, &glb);
}

static term rgba(avm_int_t w, avm_int_t h, term bin)
{
    return tuple(4, atom("rgba8888"), term_from_int(w), term_from_int(h), bin);
}

static term flips(bool flip_x, bool flip_y)
{
    term opts = term_nil();
    if (flip_y) {
        opts = cons(tuple(2, atom("flip_y"), TRUE_ATOM), opts);
    }
    if (flip_x) {
        opts = cons(tuple(2, atom("flip_x"), TRUE_ATOM), opts);
    }
    return opts;
}

static term sci(avm_int_t w, avm_int_t h, avm_int_t sx, avm_int_t sy, avm_int_t xs, avm_int_t ys,
    term opts, term img)
{
    return tuple(12, atom("scaled_cropped_image"), term_from_int(0), term_from_int(0), term_from_int(w),
        term_from_int(h), atom("transparent"), term_from_int(sx), term_from_int(sy), term_from_int(xs), term_from_int(ys),
        opts, img);
}

static void delete_item(const BaseDisplayItem *item)
{
    BaseDisplayItem *copy = malloc(sizeof(BaseDisplayItem));
    if (copy == NULL) {
        abort();
    }
    *copy = *item;
    display_items_delete(copy, 1);
}

static bool is_zeroed(const BaseDisplayItem *item)
{
    BaseDisplayItem zero;
    memset(&zero, 0, sizeof(zero));
    return memcmp(item, &zero, sizeof(zero)) == 0;
}

static term all_items;

static void expect_invalid(const char *name, term req)
{
    long outstanding = alloc_counter_outstanding;
    BaseDisplayItem item;
    memset(&item, 0xA5, sizeof(item));
    display_items_init_item(&item, req, &ctx);
    CHECK(item.primitive == PrimitiveInvalid && is_zeroed(&item),
        "%s: expected an all-zero PrimitiveInvalid item, got primitive %d bbox (%d, %d, %d, %d)",
        name, item.primitive, item.x, item.y, item.width, item.height);
    CHECK(alloc_counter_outstanding == outstanding, "%s: %ld allocations left after parsing", name,
        alloc_counter_outstanding - outstanding);
    delete_item(&item);
    all_items = cons(req, all_items);
}

static void expect_invalid_on_alloc_failure(const char *name, term req, int allocations)
{
    for (int n = 1; n <= allocations; n++) {
        long outstanding = alloc_counter_outstanding;
        BaseDisplayItem item;
        memset(&item, 0xA5, sizeof(item));
        alloc_counter_fail_at = alloc_counter_calls + n;
        display_items_init_item(&item, req, &ctx);
        alloc_counter_fail_at = -1;
        CHECK(item.primitive == PrimitiveInvalid && is_zeroed(&item),
            "%s, allocation %d failing: primitive %d", name, n, item.primitive);
        CHECK(alloc_counter_outstanding == outstanding, "%s, allocation %d failing: %ld allocations left",
            name, n, alloc_counter_outstanding - outstanding);
        delete_item(&item);
    }
}

static void parse(term req, BaseDisplayItem *item)
{
    display_items_init_item(item, req, &ctx);
    all_items = cons(req, all_items);
}

static void render(term display_list, uint16_t *frame)
{
    BaseDisplayItem *items;
    size_t len;
    if (display_items_new_list(display_list, &items, &len, &ctx) != DisplayItemsOk) {
        fprintf(stderr, "render: display_items_new_list failed\n");
        abort();
    }

    uint16_t line[SCREEN_W];
    struct DCSLCDScreen screen;
    memset(&screen, 0, sizeof(screen));
    screen.w = SCREEN_W;
    screen.h = SCREEN_H;
    screen.pixels = line;

    for (int ypos = 0; ypos < SCREEN_H; ypos++) {
        BaseDisplayItem *row = display_items_row(items, len, ypos);
        memset(line, 0, sizeof(line));
        int xpos = 0;
        while (xpos < SCREEN_W) {
            int drawn_pixels = dcs_lcd_draw_x(&screen, xpos, ypos, row);
            if (drawn_pixels <= 0) {
                fprintf(stderr, "dcs_lcd_draw_x returned %d at (%d, %d)\n", drawn_pixels, xpos, ypos);
                abort();
            }
            xpos += drawn_pixels;
        }
        if (frame) {
            memcpy(frame + ypos * SCREEN_W, line, sizeof(line));
        }
    }

    display_items_delete(items, len);
}

static bool big_ints(void)
{
    return sizeof(avm_int_t) > sizeof(int);
}

static void test_scaled_cropped_image_invalid(void)
{
    term ok_img = rgba(4, 2, pixels(32));

    for (int sy = 2; sy <= 4; sy++) {
        expect_invalid("source_y >= image height, flip_y", sci(4, 2, 0, sy, 1, 1, flips(false, true), ok_img));
        expect_invalid("source_y >= image height", sci(4, 2, 0, sy, 1, 1, term_nil(), ok_img));
    }
    expect_invalid("source_y huge, flip_y", sci(4, 2, 0, 100000000, 1, 1, flips(false, true), ok_img));
    expect_invalid("source_x >= image width, flip_x", sci(4, 2, 4, 0, 1, 1, flips(true, false), ok_img));
    expect_invalid("source_x -1", sci(4, 1, -1, 0, 1, 1, term_nil(), ok_img));
    expect_invalid("source_y -1", sci(4, 2, 0, -1, 1, 1, term_nil(), ok_img));
    expect_invalid("image height 0, empty binary", sci(4, 1, 0, 0, 1, 1, term_nil(), rgba(4, 0, pixels(0))));
    expect_invalid("image width 0, flip_x", sci(4, 1, 0, 0, 1, 1, flips(true, false), rgba(0, 1, pixels(0))));
    expect_invalid("image width -1", sci(4, 1, 0, 0, 1, 1, term_nil(), rgba(-1, 1, pixels(16))));
    expect_invalid("binary shorter than W*H*4", sci(4, 4, 0, 0, 1, 1, term_nil(), rgba(4, 4, pixels(16))));
    expect_invalid("binary one byte short", sci(4, 2, 0, 0, 1, 1, term_nil(), rgba(4, 2, pixels(31))));
    expect_invalid("pixels not a binary", sci(4, 2, 0, 0, 1, 1, term_nil(), rgba(4, 2, term_from_int(0))));
    expect_invalid("unsupported format",
        sci(4, 2, 0, 0, 1, 1, term_nil(), tuple(4, atom("rgb565"), term_from_int(4), term_from_int(2), pixels(32))));
    expect_invalid("image tuple arity 3", sci(4, 2, 0, 0, 1, 1, term_nil(), tuple(3, atom("rgba8888"), term_from_int(4), term_from_int(2))));
    expect_invalid("image not a tuple", sci(4, 2, 0, 0, 1, 1, term_nil(), term_from_int(7)));
    expect_invalid("x_scale 0", sci(4, 2, 0, 0, 0, 1, term_nil(), ok_img));
    expect_invalid("y_scale 0", sci(4, 2, 0, 0, 1, 0, term_nil(), ok_img));
    expect_invalid("x_scale -2", sci(4, 2, 0, 0, -2, 1, term_nil(), ok_img));
    expect_invalid("width -1", sci(-1, 2, 0, 0, 1, 1, term_nil(), ok_img));
    expect_invalid("x_scale above limit", sci(4, 2, 0, 0, DISPLAY_ITEMS_COORD_LIMIT + 1, 1, term_nil(), ok_img));
    expect_invalid("x not an integer",
        tuple(12, atom("scaled_cropped_image"), atom("a"), term_from_int(0), term_from_int(4), term_from_int(2),
            atom("transparent"), term_from_int(0), term_from_int(0), term_from_int(1), term_from_int(1), term_nil(), ok_img));
    expect_invalid("bgcolor not an integer",
        tuple(12, atom("scaled_cropped_image"), term_from_int(0), term_from_int(0), term_from_int(4), term_from_int(2),
            atom("opaque"), term_from_int(0), term_from_int(0), term_from_int(1), term_from_int(1), term_nil(), ok_img));
    expect_invalid("arity 11",
        tuple(11, atom("scaled_cropped_image"), term_from_int(0), term_from_int(0), term_from_int(4), term_from_int(2),
            atom("transparent"), term_from_int(0), term_from_int(0), term_from_int(1), term_from_int(1), term_nil()));
    expect_invalid("arity 13",
        tuple(13, atom("scaled_cropped_image"), term_from_int(0), term_from_int(0), term_from_int(4), term_from_int(2),
            atom("transparent"), term_from_int(0), term_from_int(0), term_from_int(1), term_from_int(1), term_nil(), ok_img,
            term_from_int(0)));
    if (big_ints()) {
        avm_int_t wrap = ((avm_int_t) 1 << 32) + 1;
        expect_invalid("source_y 2^32 + 1", sci(4, 2, 0, wrap, 1, 1, term_nil(), ok_img));
        expect_invalid("y_scale 2^32 + 1", sci(4, 2, 0, 0, 1, wrap, term_nil(), ok_img));
        expect_invalid("image height 2^32 + 1", sci(4, 2, 0, 0, 1, 1, term_nil(), rgba(4, wrap, pixels(32))));
    }
}

static void test_image_invalid(void)
{
    term ok_img = rgba(4, 2, pixels(32));
    term transparent = atom("transparent");

    expect_invalid("image height 0, empty binary",
        tuple(5, atom("image"), term_from_int(0), term_from_int(0), transparent, rgba(4, 0, pixels(0))));
    expect_invalid("image width 0",
        tuple(5, atom("image"), term_from_int(0), term_from_int(0), transparent, rgba(0, 4, pixels(0))));
    expect_invalid("binary shorter than W*H*4",
        tuple(5, atom("image"), term_from_int(0), term_from_int(0), transparent, rgba(4, 4, pixels(16))));
    expect_invalid("pixels not a binary",
        tuple(5, atom("image"), term_from_int(0), term_from_int(0), transparent, rgba(4, 2, term_from_int(1))));
    expect_invalid("unsupported format",
        tuple(5, atom("image"), term_from_int(0), term_from_int(0), transparent,
            tuple(4, atom("rgb565"), term_from_int(4), term_from_int(2), pixels(32))));
    expect_invalid("image not a tuple",
        tuple(5, atom("image"), term_from_int(0), term_from_int(0), transparent, term_nil()));
    expect_invalid("bgcolor not an integer",
        tuple(5, atom("image"), term_from_int(0), term_from_int(0), atom("opaque"), ok_img));
    expect_invalid("arity 4", tuple(4, atom("image"), term_from_int(0), term_from_int(0), transparent));
    expect_invalid("arity 6",
        tuple(6, atom("image"), term_from_int(0), term_from_int(0), transparent, ok_img, term_from_int(0)));
    expect_invalid("x above limit",
        tuple(5, atom("image"), term_from_int(DISPLAY_ITEMS_COORD_LIMIT + 1), term_from_int(0), transparent, ok_img));
    if (big_ints()) {
        avm_int_t wrap = ((avm_int_t) 1 << 32) + 5;
        expect_invalid("x 2^32 + 5",
            tuple(5, atom("image"), term_from_int(wrap), term_from_int(0), transparent, ok_img));
        expect_invalid("image width 2^32 + 4",
            tuple(5, atom("image"), term_from_int(0), term_from_int(0), transparent, rgba(wrap - 1, 2, pixels(32))));
    }
}

static void test_command_invalid(void)
{
    expect_invalid("not a tuple", term_from_int(3));
    expect_invalid("empty tuple", tuple(0));
    expect_invalid("unknown command", tuple(2, atom("triangle"), term_from_int(1)));
}

static void test_image_valid(void)
{
    BaseDisplayItem item;
    term bin = pixels(32);
    parse(tuple(5, atom("image"), term_from_int(-1), term_from_int(3), term_from_int(0x102030), rgba(4, 2, bin)), &item);
    CHECK(item.primitive == PrimitiveImage, "image: primitive %d", item.primitive);
    CHECK(item.x == -1 && item.y == 3 && item.width == 4 && item.height == 2, "image: bbox (%d, %d, %d, %d)",
        item.x, item.y, item.width, item.height);
    CHECK(item.brcolor == 0x102030FF, "image: brcolor %#x", (unsigned) item.brcolor);
    CHECK(item.data.image_data.pix == term_binary_data(bin), "image: pixel pointer");
    delete_item(&item);

    parse(tuple(5, atom("image"), term_from_int(10), term_from_int(10), atom("transparent"), rgba(2, 2, pixels(20))), &item);
    CHECK(item.primitive == PrimitiveImage && item.brcolor == 0, "image with spare bytes: primitive %d",
        item.primitive);
    delete_item(&item);
}

static void test_scaled_cropped_image_valid(void)
{
    BaseDisplayItem item;
    term bin = pixels(32);
    parse(tuple(12, atom("scaled_cropped_image"), term_from_int(5), term_from_int(6), term_from_int(7), term_from_int(8),
              term_from_int(0xABCDEF), term_from_int(3), term_from_int(1), term_from_int(2), term_from_int(3), flips(true, true),
              rgba(4, 2, bin)),
        &item);
    CHECK(item.primitive == PrimitiveScaledCroppedImage, "sci: primitive %d", item.primitive);
    CHECK(item.x == 5 && item.y == 6 && item.width == 2 && item.height == 3, "sci: bbox (%d, %d, %d, %d)",
        item.x, item.y, item.width, item.height);
    CHECK(item.brcolor == 0xABCDEFFF, "sci: brcolor %#x", (unsigned) item.brcolor);
    CHECK(item.source_x == 3 && item.source_y == 1 && item.x_scale == 2 && item.y_scale == 3,
        "sci: source (%d, %d) scale (%d, %d)", item.source_x, item.source_y, item.x_scale, item.y_scale);
    CHECK(item.flip_x && item.flip_y, "sci: flips %d %d", item.flip_x, item.flip_y);
    CHECK(item.data.image_data_with_size.width == 4 && item.data.image_data_with_size.height == 2
            && item.data.image_data_with_size.pix == term_binary_data(bin),
        "sci: image data");
    delete_item(&item);

    parse(sci(4, 2, 0, 0, 1, 1, list(2, tuple(2, atom("flip_x"), FALSE_ATOM), term_from_int(3)), rgba(4, 2, pixels(32))),
        &item);
    CHECK(item.primitive == PrimitiveScaledCroppedImage && !item.flip_x && !item.flip_y,
        "sci with false/garbage opts: primitive %d flips %d %d", item.primitive, item.flip_x, item.flip_y);
    delete_item(&item);

    struct
    {
        const char *name;
        term opts;
        bool flip_x;
        bool flip_y;
    } opts[] = {
        { "bare atoms", list(2, atom("flip_y"), atom("flip_x")), true, true },
        { "bare flip_x", list(1, atom("flip_x")), true, false },
        { "mixed forms", list(3, atom("flip_y"), tuple(2, atom("flip_x"), TRUE_ATOM), atom("other")), true, true },
        { "{flip_y, 1}", list(1, tuple(2, atom("flip_y"), term_from_int(1))), false, false },
        { "{flip_x}", list(1, tuple(1, atom("flip_x"))), false, false },
        { "not a list", atom("flip_x"), false, false },
        { "a tuple", tuple(2, atom("flip_x"), TRUE_ATOM), false, false },
        { "improper tail", cons(atom("flip_y"), atom("flip_x")), false, true },
    };
    for (size_t i = 0; i < sizeof(opts) / sizeof(opts[0]); i++) {
        parse(sci(4, 2, 0, 0, 1, 1, opts[i].opts, rgba(4, 2, pixels(32))), &item);
        CHECK(item.primitive == PrimitiveScaledCroppedImage && item.flip_x == opts[i].flip_x
                && item.flip_y == opts[i].flip_y,
            "sci opts %s: primitive %d flips %d %d", opts[i].name, item.primitive, item.flip_x, item.flip_y);
        delete_item(&item);
    }

    parse(sci(9, 9, 3, 1, 3, 3, flips(true, true), rgba(4, 2, pixels(32))), &item);
    CHECK(item.primitive == PrimitiveScaledCroppedImage, "sci at last pixel: primitive %d", item.primitive);
    delete_item(&item);
}

static void test_integer_forms(void)
{
    BaseDisplayItem item;
    term transparent = atom("transparent");
    term ok_img = rgba(4, 2, pixels(32));

    parse(tuple(5, atom("image"), term_from_int(0), term_from_int(0), term_from_int(0x7F102030), ok_img), &item);
    CHECK(item.primitive == PrimitiveImage && item.brcolor == 0x102030FF, "color above 24 bits: brcolor %#x",
        (unsigned) item.brcolor);
    delete_item(&item);
    parse(tuple(5, atom("image"), term_from_int(0), term_from_int(0), boxed_int(((avm_int64_t) 0xABCD << 32) | 0xFF102030), ok_img),
        &item);
    CHECK(item.primitive == PrimitiveImage && item.brcolor == 0x102030FF, "boxed color: brcolor %#x",
        (unsigned) item.brcolor);
    delete_item(&item);
    parse(tuple(5, atom("image"), term_from_int(0), term_from_int(0), term_from_int(-1), ok_img), &item);
    CHECK(item.primitive == PrimitiveImage && item.brcolor == 0xFFFFFFFF, "color -1: brcolor %#x",
        (unsigned) item.brcolor);
    delete_item(&item);

    parse(tuple(5, atom("image"), boxed_int(-7), boxed_int(9), transparent, ok_img), &item);
    CHECK(item.primitive == PrimitiveImage && item.x == -7 && item.y == 9, "boxed position: primitive %d (%d, %d)",
        item.primitive, item.x, item.y);
    delete_item(&item);
    expect_invalid("image x boxed 2^31",
        tuple(5, atom("image"), boxed_int((avm_int64_t) 1 << 31), term_from_int(0), transparent, ok_img));
}

static term text_binary(const char *text)
{
    if (buffers_len == MAX_BUFFERS) {
        abort();
    }
    size_t len = strlen(text);
    char *data = malloc(len ? len : 1);
    if (data == NULL) {
        abort();
    }
    memcpy(data, text, len);
    buffers[buffers_len++] = data;
    return term_from_const_binary(data, len, &heap, &glb);
}

static term string(const char *text)
{
    term l = term_nil();
    for (int i = (int) strlen(text) - 1; i >= 0; i--) {
        l = cons(term_from_int((unsigned char) text[i]), l);
    }
    return l;
}

static term rect(avm_int_t x, avm_int_t y, avm_int_t w, avm_int_t h, avm_int_t color)
{
    return tuple(6, atom("rect"), term_from_int(x), term_from_int(y), term_from_int(w), term_from_int(h), term_from_int(color));
}

static void test_rect(void)
{
    term c = term_from_int(0x00FF00);
    BaseDisplayItem item;

    parse(rect(-3, 4, 30, 20, 0xA0B0C0), &item);
    CHECK(item.primitive == PrimitiveRect, "rect: primitive %d", item.primitive);
    CHECK(item.x == -3 && item.y == 4 && item.width == 30 && item.height == 20, "rect: bbox (%d, %d, %d, %d)",
        item.x, item.y, item.width, item.height);
    CHECK(item.brcolor == 0xA0B0C0FF, "rect: brcolor %#x", (unsigned) item.brcolor);
    delete_item(&item);

    const int L = DISPLAY_ITEMS_COORD_LIMIT;
    avm_int64_t huge = (avm_int64_t) 1 << 40;
    struct
    {
        term x, y, w, h;
        int cx, cy, cw, ch;
    } clamps[] = {
        { term_from_int(0), term_from_int(0), term_from_int(100000), term_from_int(100000), 0, 0, L, L },
        { term_from_int(-100000), term_from_int(-50000), term_from_int(200000), term_from_int(50100), -L, -L, 2 * L, L + 100 },
        { term_from_int(L), term_from_int(-L), term_from_int(L), term_from_int(-L), L, -L, 0, 0 },
        { term_from_int(L + 1), term_from_int(-L - 1), term_from_int(4), term_from_int(4), L, -L, 0, 3 },
        { term_from_int(3), term_from_int(4), term_from_int(-5), term_from_int(0), 3, 4, 0, 0 },
        { boxed_int(-huge), boxed_int(huge), boxed_int(2 * huge), term_from_int(1), -L, L, 2 * L, 0 },
        { boxed_int(INT64_MAX), boxed_int(INT64_MIN), boxed_int(INT64_MAX), boxed_int(INT64_MAX), L, -L, 0, L - 1 },
    };
    for (size_t i = 0; i < sizeof(clamps) / sizeof(clamps[0]); i++) {
        parse(tuple(6, atom("rect"), clamps[i].x, clamps[i].y, clamps[i].w, clamps[i].h, c), &item);
        CHECK(item.primitive == PrimitiveRect && item.x == clamps[i].cx && item.y == clamps[i].cy
                && item.width == clamps[i].cw && item.height == clamps[i].ch,
            "clamped rect %zu: primitive %d bbox (%d, %d, %d, %d)", i, item.primitive, item.x, item.y, item.width,
            item.height);
        delete_item(&item);
    }

    expect_invalid("{rect}", tuple(1, atom("rect")));
    expect_invalid("rect arity 5", tuple(5, atom("rect"), term_from_int(0), term_from_int(0), term_from_int(4), term_from_int(4)));
    expect_invalid("rect arity 7",
        tuple(7, atom("rect"), term_from_int(0), term_from_int(0), term_from_int(4), term_from_int(4), c, c));
    expect_invalid("rect x not an integer",
        tuple(6, atom("rect"), atom("a"), term_from_int(0), term_from_int(4), term_from_int(4), c));
    expect_invalid("rect height not an integer",
        tuple(6, atom("rect"), term_from_int(0), term_from_int(0), term_from_int(4), term_nil(), c));
    expect_invalid("rect color not an integer",
        tuple(6, atom("rect"), term_from_int(0), term_from_int(0), term_from_int(4), term_from_int(4), atom("red")));
}

static term text(term x, term font, term fg, term bg, term str)
{
    return tuple(7, atom("text"), x, term_from_int(5), font, fg, bg, str);
}

static void test_text(void)
{
    term font = atom("default16px");
    term fg = term_from_int(0x102030);
    term transparent = atom("transparent");
    BaseDisplayItem item;

    parse(text(term_from_int(-4), font, fg, transparent, text_binary("hi")), &item);
    CHECK(item.primitive == PrimitiveText, "text: primitive %d", item.primitive);
    CHECK(item.x == -4 && item.y == 5 && item.width == 16 && item.height == 16, "text: bbox (%d, %d, %d, %d)",
        item.x, item.y, item.width, item.height);
    CHECK(item.brcolor == 0 && item.data.text_data.fgcolor == 0x102030FF, "text: colors %#x %#x",
        (unsigned) item.brcolor, (unsigned) item.data.text_data.fgcolor);
    CHECK(item.data.text_data.text && strcmp(item.data.text_data.text, "hi") == 0, "text: string");
    delete_item(&item);

    parse(text(term_from_int(1), font, fg, term_from_int(0xFFEEDD), string("abc")), &item);
    CHECK(item.primitive == PrimitiveText && item.width == 24 && item.brcolor == 0xFFEEDDFF,
        "text from a list: primitive %d width %d brcolor %#x", item.primitive, item.width, (unsigned) item.brcolor);
    delete_item(&item);

    parse(text(term_from_int(1), atom("nofont"), fg, transparent, string("ab")), &item);
    CHECK(item.primitive == PrimitiveText && item.width == 16, "text with unknown font: primitive %d width %d",
        item.primitive, item.width);
    delete_item(&item);

    const int L = DISPLAY_ITEMS_COORD_LIMIT;
    parse(text(term_from_int(L + 1), font, fg, transparent, string("ab")), &item);
    CHECK(item.primitive == PrimitiveText && item.x == L && item.width == 0, "text right of the limit: (%d, %d)",
        item.x, item.width);
    delete_item(&item);
    parse(text(term_from_int(L - 10), font, fg, transparent, string("abc")), &item);
    CHECK(item.primitive == PrimitiveText && item.x == L - 10 && item.width == 10, "text across the limit: (%d, %d)",
        item.x, item.width);
    delete_item(&item);
    parse(tuple(7, atom("text"), term_from_int(0), boxed_int(INT64_MIN), font, fg, transparent, string("a")), &item);
    CHECK(item.primitive == PrimitiveText && item.y == -L && item.x == 0 && item.width == 8, "text far above: (%d, %d)",
        item.x, item.y);
    delete_item(&item);
    parse(text(term_from_int(-L - 20), font, fg, transparent, string("abcdefgh")), &item);
    CHECK(item.primitive == PrimitiveText && item.x == -L + 4 && item.width == 40
            && strcmp(item.data.text_data.text, "defgh") == 0,
        "text across -L: (%d, %d) \"%s\"", item.x, item.width, item.data.text_data.text);
    delete_item(&item);
    parse(text(boxed_int(INT64_MIN), font, fg, transparent, string("ab")), &item);
    CHECK(item.primitive == PrimitiveText && item.x == -L && item.width == 0, "text far left: (%d, %d)", item.x,
        item.width);
    delete_item(&item);

    expect_invalid("{text}", tuple(1, atom("text")));
    expect_invalid("text arity 6",
        tuple(6, atom("text"), term_from_int(0), term_from_int(0), font, fg, transparent));
    expect_invalid("text arity 8",
        tuple(8, atom("text"), term_from_int(0), term_from_int(0), font, fg, transparent, string("a"), term_from_int(0)));
    expect_invalid("text x not an integer", text(atom("a"), font, fg, transparent, string("a")));
    expect_invalid("text color not an integer", text(term_from_int(0), font, atom("red"), transparent, string("a")));
    expect_invalid("text bgcolor not an integer", text(term_from_int(0), font, fg, atom("opaque"), string("a")));
    expect_invalid("text font a tuple", text(term_from_int(0), tuple(1, atom("x")), fg, transparent, string("a")));
    expect_invalid("text font an integer", text(term_from_int(0), term_from_int(16), fg, transparent, string("a")));
    expect_invalid("text font a binary", text(term_from_int(0), text_binary("default16px"), fg, transparent, string("a")));
    expect_invalid("text not a string", text(term_from_int(0), font, fg, transparent, term_from_int(3)));
    expect_invalid("text improper list", text(term_from_int(0), font, fg, transparent, cons(term_from_int('a'), term_from_int(3))));
    expect_invalid_on_alloc_failure("text", text(term_from_int(0), font, fg, transparent, string("abc")), 1);
}

static term *heap_mark(void)
{
    return heap.heap_ptr;
}

static void heap_release(term *mark)
{
    heap.heap_ptr = mark;
}

static uint16_t expected_color(const uint8_t *rgba)
{
    uint16_t c = (uint16_t) ((rgba[0] >> 3) << 11 | (rgba[1] >> 2) << 5 | (rgba[2] >> 3));
    return (uint16_t) (c >> 8 | c << 8);
}

#define OCCLUDER_COLOR 0x3060A0

static term add_occluders(term display_list, uint16_t *expected, int x, int y, int w, int h, int seed)
{
    uint8_t rgb[3] = { (OCCLUDER_COLOR >> 16) & 0xFF, (OCCLUDER_COLOR >> 8) & 0xFF, OCCLUDER_COLOR & 0xFF };
    uint16_t color = expected_color(rgb);
    for (int r = 0; r < h; r++) {
        if ((r + seed) % 4 == 3) {
            continue;
        }
        int rx = x + (r * 5 + seed) % w;
        int rw = 1 + (r + seed) % 3;
        display_list = cons(rect(rx, y + r, rw, 1, OCCLUDER_COLOR), display_list);
        for (int px = rx; px < rx + rw; px++) {
            if (px >= 0 && px < SCREEN_W && y + r >= 0 && y + r < SCREEN_H) {
                expected[(y + r) * SCREEN_W + px] = color;
            }
        }
    }
    return display_list;
}

static void compare_frames(const char *name, const uint16_t *expected, const uint16_t *frame)
{
    int wrong = 0;
    int first = -1;
    for (int i = 0; i < SCREEN_W * SCREEN_H; i++) {
        if (expected[i] != frame[i]) {
            if (first < 0) {
                first = i;
            }
            wrong++;
        }
    }
    CHECK(wrong == 0, "%s: %d pixels differ, first at (%d, %d): %#06x, expected %#06x", name, wrong,
        first % SCREEN_W, first / SCREEN_W, frame[first < 0 ? 0 : first], expected[first < 0 ? 0 : first]);
}

static void check_image_pixels(int x, int y, int w, int h, int seed)
{
    static uint16_t expected[SCREEN_W * SCREEN_H];
    static uint16_t frame[SCREEN_W * SCREEN_H];
    term *mark = heap_mark();
    term bin = pixels(w * h * 4);
    const uint8_t *bytes = (const uint8_t *) term_binary_data(bin);

    memset(expected, 0, sizeof(expected));
    for (int py = 0; py < h; py++) {
        for (int px = 0; px < w; px++) {
            if (x + px >= 0 && x + px < SCREEN_W && y + py >= 0 && y + py < SCREEN_H) {
                expected[(y + py) * SCREEN_W + x + px] = expected_color(bytes + 4 * (py * w + px));
            }
        }
    }
    term display_list = list(1,
        tuple(5, atom("image"), term_from_int(x), term_from_int(y), atom("transparent"), rgba(w, h, bin)));
    display_list = add_occluders(display_list, expected, x, y, w, h, seed);
    render(display_list, frame);

    char name[96];
    snprintf(name, sizeof(name), "image %dx%d at (%d, %d)", w, h, x, y);
    compare_frames(name, expected, frame);
    heap_release(mark);
}

static FILE *capture;
static int saved_stderr;

static void begin_capture(void)
{
    capture = tmpfile();
    if (capture == NULL) {
        abort();
    }
    fflush(stderr);
    saved_stderr = dup(STDERR_FILENO);
    dup2(fileno(capture), STDERR_FILENO);
}

static void end_capture(char *out, size_t out_size)
{
    fflush(stderr);
    dup2(saved_stderr, STDERR_FILENO);
    close(saved_stderr);
    rewind(capture);
    size_t n = fread(out, 1, out_size - 1, capture);
    out[n] = 0;
    fclose(capture);
}

static display_items_result_t capture_new_list(term display_list, char *out, size_t out_size)
{
    BaseDisplayItem *items = NULL;
    size_t len = 0;
    begin_capture();
    display_items_result_t result = display_items_new_list(display_list, &items, &len, &ctx);
    end_capture(out, out_size);
    if (result == DisplayItemsOk) {
        display_items_delete(items, len);
    }

    return result;
}

static void test_log_format(void)
{
    static char log[4096];
    term *mark = heap_mark();
    term ok = rect(0, 0, 4, 4, 0);
    term bad_rect = tuple(5, atom("rect"), term_from_int(0), term_from_int(0), term_from_int(4), term_from_int(4));

    capture_new_list(list(4, ok, bad_rect, term_from_int(3), tuple(2, term_from_int(1), term_from_int(2))), log, sizeof(log));
    const char *expected = "invalid display list item 2 (rect/5): wrong arity\n"
                           "invalid display list item 3: not a command tuple\n"
                           "invalid display list item 4 (tuple/2): unknown command\n";
    CHECK(strcmp(log, expected) == 0, "log of 3 invalid items:\n%s", log);

    capture_new_list(list(6, tuple(0), ok, tuple(1, atom("triangle")), bad_rect, bad_rect, bad_rect), log,
        sizeof(log));
    expected = "invalid display list item 1 (tuple/0): not a command tuple\n"
               "invalid display list item 3 (triangle/1): unknown command\n"
               "invalid display list item 4 (rect/5): wrong arity\n"
               "2 more invalid display list items\n";
    CHECK(strcmp(log, expected) == 0, "log of 5 invalid items:\n%s", log);

    capture_new_list(list(2, ok, ok), log, sizeof(log));
    CHECK(log[0] == 0, "log of a valid list:\n%s", log);

    term many = term_nil();
    for (int i = 0; i < 40; i++) {
        many = cons(term_from_int(i), many);
    }
    capture_new_list(many, log, sizeof(log));
    expected = "invalid display list item 1: not a command tuple\n"
               "invalid display list item 2: not a command tuple\n"
               "invalid display list item 3: not a command tuple\n"
               "37 more invalid display list items\n";
    CHECK(strcmp(log, expected) == 0, "log of 40 invalid items:\n%s", log);

    BaseDisplayItem item;
    begin_capture();
    display_items_init_item(&item, bad_rect, &ctx);
    end_capture(log, sizeof(log));
    CHECK(strcmp(log, "invalid display list item (rect/5): wrong arity\n") == 0, "log of one item:\n%s", log);
    heap_release(mark);
}

static void test_new_list(void)
{
    static char log[4096];
    term *mark = heap_mark();
    term ok = rect(0, 0, 4, 4, 0);
    BaseDisplayItem *items = (BaseDisplayItem *) 1;
    size_t len = 1;

    long outstanding = alloc_counter_outstanding;
    display_items_result_t result = display_items_new_list(term_nil(), &items, &len, &ctx);
    CHECK(result == DisplayItemsOk && items == NULL && len == 0, "empty list: result %d, %zu items", result, len);
    CHECK(alloc_counter_outstanding == outstanding, "empty list: %ld allocations",
        alloc_counter_outstanding - outstanding);
    display_items_delete(items, len);

    result = display_items_new_list(list(2, ok, ok), &items, &len, &ctx);
    CHECK(result == DisplayItemsOk && items != NULL && len == 2 && items[1].primitive == PrimitiveRect,
        "list of 2: result %d, %zu items", result, len);
    display_items_delete(items, len);

    result = capture_new_list(cons(ok, term_from_int(1)), log, sizeof(log));
    CHECK(result == DisplayItemsNotAProperList && strcmp(log, "invalid display list: not a proper list\n") == 0,
        "improper list: result %d, log:\n%s", result, log);
    result = capture_new_list(ok, log, sizeof(log));
    CHECK(result == DisplayItemsNotAProperList, "tuple instead of a list: result %d", result);
    result = capture_new_list(atom("rect"), log, sizeof(log));
    CHECK(result == DisplayItemsNotAProperList, "atom instead of a list: result %d", result);

    alloc_counter_fail_at = alloc_counter_calls + 1;
    result = capture_new_list(list(1, ok), log, sizeof(log));
    alloc_counter_fail_at = -1;
    CHECK(result == DisplayItemsOutOfMemory && strcmp(log, "failed to allocate display list items\n") == 0,
        "allocation failure: result %d, log:\n%s", result, log);
    CHECK(alloc_counter_outstanding == outstanding, "after errors: %ld allocations",
        alloc_counter_outstanding - outstanding);
    heap_release(mark);
}

static void test_huge_rect_pixels(void)
{
    static uint16_t expected[SCREEN_W * SCREEN_H];
    static uint16_t frame[SCREEN_W * SCREEN_H];
    const uint8_t rgb[3] = { 0x12, 0x34, 0x56 };
    term *mark = heap_mark();
    for (int i = 0; i < SCREEN_W * SCREEN_H; i++) {
        expected[i] = expected_color(rgb);
    }
    render(list(1, rect(0, 0, 100000, 100000, 0x123456)), frame);
    compare_frames("rect 100000x100000", expected, frame);
    avm_int64_t huge = (avm_int64_t) 1 << 40;
    term req = tuple(6, atom("rect"), boxed_int(-huge), term_from_int(-100000), boxed_int(2 * huge), term_from_int(200000),
        term_from_int(0x123456));
    render(list(1, req), frame);
    compare_frames("rect from -2^40 to 2^40", expected, frame);
    heap_release(mark);
}

static void test_image_pixels(void)
{
    check_image_pixels(3, 2, 5, 4, 0);
    check_image_pixels(-2, -1, 7, 5, 1);
    check_image_pixels(SCREEN_W - 4, SCREEN_H - 3, 6, 5, 2);
}

struct Sprite
{
    int x;
    int y;
    int width;
    int height;
    int source_x;
    int source_y;
    int x_scale;
    int y_scale;
    bool flip_x;
    bool flip_y;
    int img_width;
    int img_height;
    const uint8_t *bytes;
};

static int min_int(int a, int b)
{
    return (a < b) ? a : b;
}

static int sprite_drawn_width(const struct Sprite *s)
{
    return min_int(s->width, (s->img_width - s->source_x) * s->x_scale);
}

static int sprite_drawn_height(const struct Sprite *s)
{
    return min_int(s->height, (s->img_height - s->source_y) * s->y_scale);
}

static bool sprite_pixel(const struct Sprite *s, int px, int py, uint16_t *color)
{
    int drawn_w = sprite_drawn_width(s);
    int drawn_h = sprite_drawn_height(s);
    if (px < 0 || py < 0 || px >= drawn_w || py >= drawn_h) {
        return false;
    }
    if (s->flip_x) {
        px = drawn_w - 1 - px;
    }
    if (s->flip_y) {
        py = drawn_h - 1 - py;
    }
    int c = px / s->x_scale;
    int r = py / s->y_scale;
    *color = expected_color(s->bytes + 4 * ((s->source_y + r) * s->img_width + s->source_x + c));
    return true;
}

static void check_sprite_pixels(const struct Sprite *s, int seed)
{
    static uint16_t expected[SCREEN_W * SCREEN_H];
    static uint16_t frame[SCREEN_W * SCREEN_H];
    char name[160];
    snprintf(name, sizeof(name),
        "sprite %dx%d from (%d, %d) of %dx%d, scale %dx%d, flip %d %d, at (%d, %d)", s->width, s->height,
        s->source_x, s->source_y, s->img_width, s->img_height, s->x_scale, s->y_scale, s->flip_x, s->flip_y,
        s->x, s->y);

    memset(expected, 0, sizeof(expected));
    for (int y = 0; y < SCREEN_H; y++) {
        for (int x = 0; x < SCREEN_W; x++) {
            sprite_pixel(s, x - s->x, y - s->y, &expected[y * SCREEN_W + x]);
        }
    }

    term *mark = heap_mark();
    term bin = term_from_const_binary(s->bytes, s->img_width * s->img_height * 4, &heap, &glb);
    term req = tuple(12, atom("scaled_cropped_image"), term_from_int(s->x), term_from_int(s->y), term_from_int(s->width),
        term_from_int(s->height), atom("transparent"), term_from_int(s->source_x), term_from_int(s->source_y),
        term_from_int(s->x_scale), term_from_int(s->y_scale), flips(s->flip_x, s->flip_y),
        rgba(s->img_width, s->img_height, bin));

    BaseDisplayItem item;
    display_items_init_item(&item, req, &ctx);
    CHECK(item.primitive == PrimitiveScaledCroppedImage && item.width == sprite_drawn_width(s)
            && item.height == sprite_drawn_height(s),
        "%s: primitive %d, size %dx%d, expected %dx%d", name, item.primitive, item.width, item.height,
        sprite_drawn_width(s), sprite_drawn_height(s));
    delete_item(&item);

    term display_list = add_occluders(list(1, req), expected, s->x, s->y, sprite_drawn_width(s),
        sprite_drawn_height(s), seed);
    render(display_list, frame);
    compare_frames(name, expected, frame);
    heap_release(mark);
}

static void test_sprite_pixels(void)
{
    enum
    {
        ImgW = 6,
        ImgH = 5
    };
    term bin = pixels(ImgW * ImgH * 4);
    const uint8_t *bytes = (const uint8_t *) term_binary_data(bin);
    int seed = 0;

    for (int sx = 0; sx <= 2; sx += 2) {
        for (int sy = 0; sy <= 1; sy++) {
            for (int crop = 0; crop < 3; crop++) {
                for (int xs = 1; xs <= 3; xs++) {
                    for (int ys = 1; ys <= 3; ys++) {
                        for (int f = 0; f < 4; f++) {
                            int rest_w = ImgW - sx;
                            int rest_h = ImgH - sy;
                            struct Sprite s;
                            memset(&s, 0, sizeof(s));
                            s.img_width = ImgW;
                            s.img_height = ImgH;
                            s.bytes = bytes;
                            s.source_x = sx;
                            s.source_y = sy;
                            s.x_scale = xs;
                            s.y_scale = ys;
                            s.flip_x = f & 1;
                            s.flip_y = f & 2;
                            if (crop == 0) {
                                s.width = (rest_w - 2) * xs - (xs - 1);
                                s.height = (rest_h - 2) * ys - (ys - 1);
                            } else if (crop == 1) {
                                s.width = rest_w * xs;
                                s.height = rest_h * ys;
                            } else {
                                s.width = rest_w * xs + 3;
                                s.height = rest_h * ys + 2 * ys + 1;
                            }
                            s.x = (seed % 5 == 4) ? -3 : 2 + seed % 7;
                            s.y = (seed % 5 == 4) ? -2 : 1 + seed % 5;
                            check_sprite_pixels(&s, seed);
                            seed++;
                        }
                    }
                }
            }
        }
    }
}

int main(void)
{
    heap.heap_start = heap_buf;
    heap.heap_ptr = heap_buf;
    heap.heap_end = heap_buf + HEAP_TERMS;
    ctx.global = &glb;
    all_items = term_nil();

    test_scaled_cropped_image_invalid();
    test_image_invalid();
    test_command_invalid();
    test_image_valid();
    test_scaled_cropped_image_valid();
    test_integer_forms();
    test_rect();
    test_text();
    test_log_format();
    test_new_list();
    test_huge_rect_pixels();
    test_image_pixels();
    test_sprite_pixels();

    render(all_items, NULL);

    CHECK(term_display_non_atoms == 0, "%d terms other than atoms were logged", term_display_non_atoms);

    for (int i = 0; i < buffers_len; i++) {
        free(buffers[i]);
    }
    CHECK(alloc_counter_outstanding == 0, "%ld allocations were not freed", alloc_counter_outstanding);

    if (failures) {
        fprintf(stderr, "%d of %d checks failed\n", failures, checks);
        return 1;
    }
    printf("all %d display item checks passed\n", checks);
    return 0;
}
