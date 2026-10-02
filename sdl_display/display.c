/*
 * This file is part of AtomGL.
 *
 * Copyright 2021-2023 Davide Bettio <davide@uninstall.it>
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

#include <SDL.h>
#include <limits.h>
#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <context.h>
#include <defaultatoms.h>
#include <interop.h>
#include <mailbox.h>
#include <port.h>
#include <term.h>
#include <utils.h>

#include "../ufontlib.h"

#define SCREEN_WIDTH 320
#define SCREEN_HEIGHT 240
#define BPP 4
#define DEPTH 32

#include "../display_items.h"
#include "../display_message.h"
#include "../font_data.h"
#include "../image_helpers.h"

struct DisplayOpts
{
    avm_int_t width;
    avm_int_t height;
};

struct KeyboardEvent
{
    uint16_t key;
    uint16_t unicode;
    bool key_down;
};

struct MouseEvent
{
    int type;
    int button;
    int x;
    int y;
};

static term keyboard_pid;
static struct timespec ts0;
Context *the_ctx;

struct Screen
{
    int w;
    int h;
    int scale;
    void *pixels;
    SDL_PixelFormat *format;
};

static struct Screen *screen;
static SDL_Surface *surface;
static pthread_mutex_t ready_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t ready = PTHREAD_COND_INITIALIZER;

UFontManager *ufont_manager;

static NativeHandlerResult consume_display_mailbox(Context *ctx);
static void *display_loop();

static void destroy_message(Message *m, GlobalContext *global)
{
    BEGIN_WITH_STACK_HEAP(1, temp_heap);
    mailbox_message_dispose(&m->base, &temp_heap);
    END_WITH_STACK_HEAP(temp_heap, global);
}

static inline Uint32 uint32_color_to_surface(struct Screen *screen, uint32_t color)
{
    return SDL_MapRGB(screen->format, (color >> 24) & 0xFF, (color >> 16) & 0xFF, (color >> 8) & 0xFF);
}

static int draw_image_x(int xpos, int ypos, int max_line_len, BaseDisplayItem *item)
{
    int x = item->x;
    int y = item->y;

    Uint32 bgcolor;
    bool visible_bg;
    if (item->brcolor != 0) {
        bgcolor = uint32_color_to_surface(screen, item->brcolor);
        visible_bg = true;
    } else {
        visible_bg = false;
    }

    int width = item->width;
    const char *data = item->data.image_data.pix;

    int drawn_pixels = 0;

    uint32_t *pixels = ((uint32_t *) data) + (ypos - y) * width + (xpos - x);
    Uint32 *pixmem32 = (Uint32 *) (((uint8_t *) screen->pixels) + screen->w * ypos * BPP + xpos * BPP);

    if (width > xpos - x + max_line_len) {
        width = xpos - x + max_line_len;
    }

    for (int j = xpos - x; j < width; j++) {
        uint32_t img_pixel = READ_32_UNALIGNED(pixels);
        if ((*pixels >> 24) & 0xFF) {
            Uint32 color = uint32_color_to_surface(screen, img_pixel);
            pixmem32[drawn_pixels] = color;
        } else if (visible_bg) {
            pixmem32[drawn_pixels] = bgcolor;
        } else {
            return drawn_pixels;
        }
        drawn_pixels++;
        pixels++;
    }

    return drawn_pixels;
}

static int draw_scaled_cropped_img_x(int xpos, int ypos, int max_line_len, BaseDisplayItem *item)
{
    int x = item->x;
    int y = item->y;

    Uint32 bgcolor;
    bool visible_bg;
    if (item->brcolor != 0) {
        bgcolor = uint32_color_to_surface(screen, item->brcolor);
        visible_bg = true;
    } else {
        visible_bg = false;
    }

    int width = item->width;

    int drawn_pixels = 0;

    Uint32 *pixmem32 = (Uint32 *) (((uint8_t *) screen->pixels) + screen->w * ypos * BPP + xpos * BPP);

    if (width > xpos - x + max_line_len) {
        width = xpos - x + max_line_len;
    }

    struct ScaledCroppedRow src;
    display_items_scaled_cropped_row_init(&src, item, ypos - y);

    for (int j = xpos - x; j < width; j++) {
        const uint32_t *pixels = display_items_scaled_cropped_row_pixel(&src, j);
        uint32_t img_pixel = READ_32_UNALIGNED(pixels);
        if (rgba8888_get_alpha(img_pixel) != 0) {
            Uint32 color = uint32_color_to_surface(screen, img_pixel);
            pixmem32[drawn_pixels] = color;
        } else if (visible_bg) {
            pixmem32[drawn_pixels] = bgcolor;
        } else {
            return drawn_pixels;
        }
        drawn_pixels++;
    }

    return drawn_pixels;
}

static int draw_rect_x(int xpos, int ypos, int max_line_len, BaseDisplayItem *item)
{
    int x = item->x;
    int width = item->width;
    uint32_t color = uint32_color_to_surface(screen, item->brcolor);

    int drawn_pixels = 0;

    Uint32 *pixmem32 = (Uint32 *) (((uint8_t *) screen->pixels) + screen->w * ypos * BPP + xpos * BPP);

    if (width > xpos - x + max_line_len) {
        width = xpos - x + max_line_len;
    }

    for (int j = xpos - x; j < width; j++) {
        pixmem32[drawn_pixels] = color;
        drawn_pixels++;
    }

    return drawn_pixels;
}

static int draw_shape_x(int xpos, int ypos, int max_line_len, BaseDisplayItem *item,
    int *outside_run)
{
    if (display_items_shape_outside_run(item, xpos, ypos, outside_run)) {
        return 0;
    }
    bool inside;
    int run = shape_run(item->data.shape_data.shape, xpos, ypos, &inside);
    if (!inside) {
        display_items_shape_remember_outside(item, xpos, ypos, run);
        *outside_run = run;
        return 0;
    }
    if (run > max_line_len) {
        run = max_line_len;
    }
    return draw_rect_x(xpos, ypos, run, item);
}

static int draw_text_x(int xpos, int ypos, int max_line_len, BaseDisplayItem *item)
{
    int x = item->x;
    int y = item->y;
    Uint32 fgcolor = uint32_color_to_surface(screen, item->data.text_data.fgcolor);
    Uint32 bgcolor;
    bool visible_bg;
    if (item->brcolor != 0) {
        bgcolor = uint32_color_to_surface(screen, item->brcolor);
        visible_bg = true;
    } else {
        visible_bg = false;
    }

    char *text = (char *) item->data.text_data.text;

    int width = item->width;

    int drawn_pixels = 0;

    Uint32 *pixmem32 = (Uint32 *) (((uint8_t *) screen->pixels) + screen->w * ypos * BPP + xpos * BPP);

    if (width > xpos - x + max_line_len) {
        width = xpos - x + max_line_len;
    }

    for (int j = xpos - x; j < width; j++) {
        int char_index = j / CHAR_WIDTH;
        char c = text[char_index];
        unsigned const char *glyph = fontdata + ((unsigned char) c) * 16;

        unsigned char row = glyph[ypos - y];

        bool opaque;
        int k = j % CHAR_WIDTH;
        if (row & (1 << (7 - k))) {
            opaque = true;
        } else {
            opaque = false;
        }

        if (opaque) {
            pixmem32[drawn_pixels] = fgcolor;
        } else if (visible_bg) {
            pixmem32[drawn_pixels] = bgcolor;
        } else {
            return drawn_pixels;
        }
        drawn_pixels++;
    }

    return drawn_pixels;
}

static int draw_x(int xpos, int ypos, BaseDisplayItem *row)
{
    int line_len = screen->w - xpos;
    int transparent_run = INT_MAX;

    for (BaseDisplayItem *item = row; item != NULL; item = item->next) {
        if (xpos < item->x) {
            int len_to_item = item->x - xpos;
            if (len_to_item < line_len) {
                line_len = len_to_item;
            }
            continue;
        }
        if (xpos >= item->x + item->width) {
            continue;
        }

        int max_line_len = (line_len < transparent_run) ? line_len : transparent_run;

        int run = 1;
        int drawn_pixels = 0;
        switch (item->primitive) {
            case PrimitiveImage:
                drawn_pixels = draw_image_x(xpos, ypos, max_line_len, item);
                break;

            case PrimitiveScaledCroppedImage:
                drawn_pixels = draw_scaled_cropped_img_x(xpos, ypos, max_line_len, item);
                break;

            case PrimitiveRect:
                drawn_pixels = draw_rect_x(xpos, ypos, max_line_len, item);
                break;

            case PrimitiveText:
                drawn_pixels = draw_text_x(xpos, ypos, max_line_len, item);
                break;

            case PrimitiveShape:
                drawn_pixels = draw_shape_x(xpos, ypos, max_line_len, item, &run);
                break;

            default: {
                fprintf(stderr, "unexpected display list command.\n");
            }
        }

        if (drawn_pixels != 0) {
            return drawn_pixels;
        }

        if (run < transparent_run) {
            transparent_run = run;
        }
    }

    return 1;
}

static void do_update(Context *ctx, term display_list)
{
    BaseDisplayItem *items;
    size_t len;
    if (UNLIKELY(display_items_new_list(display_list, &items, &len, ctx) != DisplayItemsOk)) {
        return;
    }

    for (int ypos = 0; ypos < screen->h; ypos++) {
        BaseDisplayItem *row = display_items_row(items, len, ypos);
        int xpos = 0;
        while (xpos < screen->w) {
            xpos += draw_x(xpos, ypos, row);
        }
    }

    display_items_delete(items, len);
}

static void process_message(Context *ctx)
{
    MailboxMessage *mbox_msg = mailbox_take_message(&ctx->mailbox);
    Message *message = CONTAINER_OF(mbox_msg, Message, base);

    GenMessage gen_message;
    if (UNLIKELY(port_parse_gen_message(message->message, &gen_message) != GenCallMessage)) {
        goto invalid_message;
    }

    term req = gen_message.req;
    if (UNLIKELY(!term_is_tuple(req) || term_get_tuple_arity(req) < 1)) {
        goto invalid_message;
    }

    term cmd = term_get_tuple_element(req, 0);

    if (SDL_MUSTLOCK(surface)) {
        if (SDL_LockSurface(surface) < 0) {
            return;
        }
    }

    if (cmd == globalcontext_make_atom(ctx->global, "\x6"
                                      "update")) {
        term display_list = term_get_tuple_element(req, 1);
        do_update(ctx, display_list);

        // Copy and scale up
        int scale = screen->scale;
        for (int ypos = 0; ypos < surface->h; ypos++) {
            for (int xpos = 0; xpos < surface->w; xpos++) {
                Uint32 *srcpix = (Uint32 *) (((uint8_t *) screen->pixels) + screen->w * (ypos / scale) * BPP + (xpos / scale) * BPP);
                Uint32 *destpix = (Uint32 *) (((uint8_t *) surface->pixels) + surface->w * ypos * BPP + xpos * BPP);
                *destpix = *srcpix;
            }
        }

    } else if (cmd == globalcontext_make_atom(ctx->global, "\xF"
                                             "subscribe_input")) {
        if (term_get_tuple_arity(req) != 2) {
            goto invalid_message;
        }
        term sources = term_get_tuple_element(req, 1);
        if (term_is_pid(keyboard_pid) || sources != globalcontext_make_atom(ctx->global, "\x3" "all")) {
            fprintf(stderr, "Warning: only one subscriber to all sources is supported now\n");
        }
        // TODO: selective subscribe
        keyboard_pid = gen_message.pid;

    } else if (cmd == globalcontext_make_atom(ctx->global, "\xA" "load_image")) {

        handle_load_image(req, gen_message.ref, gen_message.pid, ctx);

        goto free_msg_and_exit;

    } else if (cmd == globalcontext_make_atom(ctx->global, "\xD" "register_font")) {
        term font_bin = term_get_tuple_element(req, 2);
        size_t font_size = term_binary_size(font_bin);
        void *owned_buf = malloc(font_size);
        EpdFont *loaded_font = NULL;
        if (owned_buf != NULL) {
            memcpy(owned_buf, term_binary_data(font_bin), font_size);
            loaded_font = ufont_parse(owned_buf, font_size);
            if (loaded_font == NULL) {
                free(owned_buf);
            }
        }

        char *handle = interop_atom_to_string(ctx, term_get_tuple_element(req, 1));
        if (loaded_font != NULL && handle != NULL) {
            ufont_manager_register(ufont_manager, handle, loaded_font, owned_buf);
        }
        free(handle);

    } else {
        fprintf(stderr, "unexpected command: ");
        term_display(stderr, req, ctx);
        fprintf(stderr, "\n");
    }

    if (SDL_MUSTLOCK(surface)) {
        SDL_UnlockSurface(surface);
    }

    SDL_Flip(surface);

    if (UNLIKELY(memory_ensure_free(ctx, TUPLE_SIZE(3)) != MEMORY_GC_OK)) {
        abort();
    }
    term return_tuple = term_alloc_tuple(2, &ctx->heap);
    term_put_tuple_element(return_tuple, 0, gen_message.ref);
    term_put_tuple_element(return_tuple, 1, OK_ATOM);

    int local_process_id = term_to_local_process_id(gen_message.pid);
    globalcontext_send_message(ctx->global, local_process_id, return_tuple);

    goto free_msg_and_exit;

invalid_message:
    fprintf(stderr, "Got invalid message: ");
    term_display(stderr, message->message, ctx);
    fprintf(stderr, "\n");
    fprintf(stderr, "Expected gen_server call.\n");

free_msg_and_exit:
    destroy_message(message, ctx->global);
    return;
}

static NativeHandlerResult consume_display_mailbox(Context *ctx)
{
    process_message(ctx);

    return NativeContinue;
}

static inline int replace_new_line(int c)
{
    return c == '\r' ? '\n' : c;
}

void send_keyboard_event(struct KeyboardEvent *keyb, Context *ctx)
{
    GlobalContext *glb = ctx->global;

    if (keyboard_pid) {
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);

        avm_int_t millis = (ts.tv_sec - ts0.tv_sec) * 1000 + (ts.tv_nsec - ts0.tv_nsec) / 1000000;

        BEGIN_WITH_STACK_HEAP(TUPLE_SIZE(3) + TUPLE_SIZE(4), heap);

        term up_down = keyb->key_down ? globalcontext_make_atom(glb, ATOM_STR("\x4", "down"))
                                      : globalcontext_make_atom(glb, ATOM_STR("\x2", "up"));
        bool supress_key = false;
        term code_or_special;
        // unicode is valid only during key down
        if (keyb->unicode) {
            code_or_special = term_from_int(replace_new_line(keyb->unicode));
        } else {
            switch (keyb->key) {
                case 274:
                    code_or_special = globalcontext_make_atom(glb, ATOM_STR("\x4", "down"));
                    break;
                case 276:
                    code_or_special = globalcontext_make_atom(glb, ATOM_STR("\x4", "left"));
                    break;
                case 273:
                    code_or_special = globalcontext_make_atom(glb, ATOM_STR("\x2", "up"));
                    break;
                case 275:
                    code_or_special = globalcontext_make_atom(glb, ATOM_STR("\x5", "right"));
                    break;
                case 301:
                    code_or_special = globalcontext_make_atom(glb, ATOM_STR("\x9", "caps_lock"));
                    break;
                case 303:
                    code_or_special = globalcontext_make_atom(glb, ATOM_STR("\xB", "right_shift"));
                    break;
                case 304:
                    code_or_special = globalcontext_make_atom(glb, ATOM_STR("\x5", "shift"));
                    break;
                case 306:
                    code_or_special = globalcontext_make_atom(glb, ATOM_STR("\x4", "ctrl"));
                    break;
                case 308:
                    code_or_special = globalcontext_make_atom(glb, ATOM_STR("\x3", "alt"));
                    break;
                case 313:
                    code_or_special = globalcontext_make_atom(glb, ATOM_STR("\x5", "altgr"));
                    break;
                default:
                    if (keyb->key <= 127) {
                        code_or_special = term_from_int(replace_new_line(keyb->key));
                    } else {
                        fprintf(stderr, "Ignoring key: %i\n", (int) keyb->key);
                        supress_key = true;
                    }
            }
        }

        if (supress_key) {
            return;
        }

        term event_data_tuple = term_alloc_tuple(3, &heap);
        term_put_tuple_element(event_data_tuple, 0, globalcontext_make_atom(glb, "\x8"
                                                                           "keyboard"));
        term_put_tuple_element(event_data_tuple, 1, up_down);
        term_put_tuple_element(event_data_tuple, 2, code_or_special);

        term event_tuple = term_alloc_tuple(4, &heap);
        term_put_tuple_element(event_tuple, 0, globalcontext_make_atom(glb, "\xB"
                                                                      "input_event"));
        term_put_tuple_element(event_tuple, 1, term_from_local_process_id(ctx->process_id));
        term_put_tuple_element(event_tuple, 2, term_from_int(millis));
        term_put_tuple_element(event_tuple, 3, event_data_tuple);

        display_message_send(keyboard_pid, event_tuple, glb);

        END_WITH_STACK_HEAP(heap, glb);
    }
}

void send_mouse_event(struct MouseEvent *mouse, Context *ctx)
{
    GlobalContext *glb = ctx->global;

    if (keyboard_pid) {
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);

        avm_int_t millis = (ts.tv_sec - ts0.tv_sec) * 1000 + (ts.tv_nsec - ts0.tv_nsec) / 1000000;

        term released = globalcontext_make_atom(glb, ATOM_STR("\x8", "released"));
        term pressed = globalcontext_make_atom(glb, ATOM_STR("\x7", "pressed"));

        bool has_state_tuple = false;
        term event_type;
        switch (mouse->type) {
            case SDL_MOUSEMOTION:
                has_state_tuple = true;
                event_type = globalcontext_make_atom(glb, ATOM_STR("\x4", "move"));
                break;
            case SDL_MOUSEBUTTONDOWN:
                event_type = pressed;
                break;
            case SDL_MOUSEBUTTONUP:
                event_type = released;
                break;
            default:
                fprintf(stderr, "Unexpected mouse event type.\n");
                return;
        };

        BEGIN_WITH_STACK_HEAP(TUPLE_SIZE(3) + TUPLE_SIZE(5) + TUPLE_SIZE(4), heap);

        term state;
        if (has_state_tuple) {
            state = term_alloc_tuple(3, &heap);
            term_put_tuple_element(state, 0, (mouse->button & SDL_BUTTON(1)) ? pressed : released);
            term_put_tuple_element(state, 1, (mouse->button & SDL_BUTTON(2)) ? pressed : released);
            term_put_tuple_element(state, 2, (mouse->button & SDL_BUTTON(3)) ? pressed : released);
        } else {
            switch (mouse->button) {
                case SDL_BUTTON_LEFT:
                    state = globalcontext_make_atom(glb, ATOM_STR("\x4", "left"));
                    break;
                case SDL_BUTTON_MIDDLE:
                    state = globalcontext_make_atom(glb, ATOM_STR("\x6", "middle"));
                    break;
                case SDL_BUTTON_RIGHT:
                    state = globalcontext_make_atom(glb, ATOM_STR("\x5", "right"));
                    break;
            }
        }

        term event_data_tuple = term_alloc_tuple(5, &heap);
        term_put_tuple_element(event_data_tuple, 0, globalcontext_make_atom(glb, ATOM_STR("\x5", "mouse")));
        term_put_tuple_element(event_data_tuple, 1, event_type);
        term_put_tuple_element(event_data_tuple, 2, state);
        term_put_tuple_element(event_data_tuple, 3, term_from_int(mouse->x));
        term_put_tuple_element(event_data_tuple, 4, term_from_int(mouse->y));

        term event_tuple = term_alloc_tuple(4, &heap);
        term_put_tuple_element(event_tuple, 0, globalcontext_make_atom(glb, ATOM_STR("\xB", "input_event")));
        term_put_tuple_element(event_tuple, 1, term_from_local_process_id(ctx->process_id));
        term_put_tuple_element(event_tuple, 2, term_from_int(millis));
        term_put_tuple_element(event_tuple, 3, event_data_tuple);

        display_message_send(keyboard_pid, event_tuple, glb);

        END_WITH_STACK_HEAP(heap, glb);
    }
}

Context *display_create_port(GlobalContext *global, term opts)
{
    Context *ctx = context_new(global);
    ctx->native_handler = consume_display_mailbox;

    term width_atom = globalcontext_make_atom(ctx->global, "\x5"
                                             "width");
    term height_atom = globalcontext_make_atom(ctx->global, "\x6"
                                              "height");

    term width_term = interop_proplist_get_value_default(opts, width_atom, term_from_int(SCREEN_WIDTH));
    term height_term = interop_proplist_get_value_default(opts, height_atom, term_from_int(SCREEN_HEIGHT));

    avm_int_t width = term_to_int(width_term);
    avm_int_t height = term_to_int(height_term);

    struct DisplayOpts *disp_opts = malloc(sizeof(struct DisplayOpts));
    if (IS_NULL_PTR(disp_opts)) {
        abort();
    }
    disp_opts->width = width;
    disp_opts->height = height;
    ctx->platform_data = disp_opts;

    UNUSED(opts);

    pthread_t thread_id;
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_create(&thread_id, &attr, display_loop, disp_opts);

    pthread_mutex_lock(&ready_mutex);
    pthread_cond_wait(&ready, &ready_mutex);
    pthread_mutex_unlock(&ready_mutex);
    the_ctx = ctx;

    clock_gettime(CLOCK_MONOTONIC, &ts0);

    return ctx;
}

static int get_scale()
{
    int scale = 1;
    const char *scale_str = getenv("AVM_SDL_DISPLAY_SCALE");
    if (scale_str && (strlen(scale_str) > 0)) {
        char *first_invalid;
        scale = strtol(scale_str, &first_invalid, 10);
        if (*first_invalid != '\0') {
            scale = 1;
        }
    }

    return scale;
}

void *display_loop(void *args)
{
    struct DisplayOpts *disp_opts = (struct DisplayOpts *) args;
    int scale = get_scale();

    pthread_mutex_lock(&ready_mutex);

    UNUSED(args);

    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        abort();
    }

    SDL_EnableUNICODE(1);

    if (!(surface = SDL_SetVideoMode(disp_opts->width * scale, disp_opts->height * scale, DEPTH, SDL_HWSURFACE))) {
        SDL_Quit();
        abort();
    }

    if (SDL_MUSTLOCK(surface)) {
        if (SDL_LockSurface(surface) < 0) {
            return NULL;
        }
    }

    screen = malloc(sizeof(struct Screen));
    screen->w = disp_opts->width;
    screen->h = disp_opts->height;
    screen->scale = scale;
    screen->pixels = malloc(disp_opts->width * disp_opts->height * BPP);
    screen->format = surface->format;

    memset(screen->pixels, 0x80, disp_opts->width * disp_opts->height * BPP);
    memset(surface->pixels, 0x80, disp_opts->width * scale * disp_opts->height * scale * BPP);

    ufont_manager = ufont_manager_new();

    if (SDL_MUSTLOCK(surface)) {
        SDL_UnlockSurface(surface);
    }

    SDL_Flip(surface);

    pthread_cond_signal(&ready);
    pthread_mutex_unlock(&ready_mutex);

    SDL_Event event;

    while (SDL_WaitEvent(&event)) {
        switch (event.type) {
            case SDL_QUIT: {
                exit(EXIT_SUCCESS);
                break;
            }

            case SDL_KEYDOWN: {
                struct KeyboardEvent keyb_event;
                memset(&keyb_event, 0, sizeof(struct KeyboardEvent));
                keyb_event.key = event.key.keysym.sym;
                keyb_event.unicode = event.key.keysym.unicode;
                keyb_event.key_down = true;
                send_keyboard_event(&keyb_event, the_ctx);
                break;
            }

            case SDL_KEYUP: {
                struct KeyboardEvent keyb_event;
                memset(&keyb_event, 0, sizeof(struct KeyboardEvent));
                keyb_event.key = event.key.keysym.sym;
                keyb_event.unicode = event.key.keysym.unicode;
                keyb_event.key_down = false;
                send_keyboard_event(&keyb_event, the_ctx);
                break;
            }

            case SDL_MOUSEMOTION: {
                struct MouseEvent mouse_event;
                memset(&mouse_event, 0, sizeof(struct MouseEvent));
                mouse_event.type = event.motion.type;
                mouse_event.button = event.motion.state;
                mouse_event.x = event.motion.x / scale;
                mouse_event.y = event.motion.y / scale;
                send_mouse_event(&mouse_event, the_ctx);
                break;
            }

            case SDL_MOUSEBUTTONDOWN: {
                struct MouseEvent mouse_event;
                memset(&mouse_event, 0, sizeof(struct MouseEvent));
                mouse_event.type = event.button.type;
                mouse_event.button = event.button.button;
                mouse_event.x = event.button.x / scale;
                mouse_event.y = event.button.y / scale;
                send_mouse_event(&mouse_event, the_ctx);
                break;
            }

            case SDL_MOUSEBUTTONUP: {
                struct MouseEvent mouse_event;
                memset(&mouse_event, 0, sizeof(struct MouseEvent));
                mouse_event.type = event.button.type;
                mouse_event.button = event.button.button;
                mouse_event.x = event.button.x / scale;
                mouse_event.y = event.button.y / scale;
                send_mouse_event(&mouse_event, the_ctx);
                break;
            }

            default: {
                break;
            }
        }
    }

    return NULL;
}
