#include "ufontlib.h"
#ifdef WITH_ZLIB
#include <zlib.h>
#else
#include "miniz.c"
#include "miniz.h"
#endif
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct
{
    uint8_t mask; /* char data will be bitwise AND with this */
    uint8_t lead; /* start bytes of current char in utf-8 encoded character */
    uint32_t beg; /* beginning of codepoint range */
    uint32_t end; /* end of codepoint range */
    int bits_stored; /* the number of bits from the codepoint that fits in char */
} utf_t;

/*
 * UTF-8 decode inspired from rosetta code
 * https://rosettacode.org/wiki/UTF-8_encode_and_decode#C
 */
static utf_t *utf[] = {
    /*             mask        lead        beg      end       bits */
    [0] = &(utf_t){ 0b00111111, 0b10000000, 0, 0, 6 },
    [1] = &(utf_t){ 0b01111111, 0b00000000, 0000, 0177, 7 },
    [2] = &(utf_t){ 0b00011111, 0b11000000, 0200, 03777, 5 },
    [3] = &(utf_t){ 0b00001111, 0b11100000, 04000, 0177777, 4 },
    [4] = &(utf_t){ 0b00000111, 0b11110000, 0200000, 04177777, 3 },
    &(utf_t){ 0 },
};

static inline int min(int x, int y) { return x < y ? x : y; }
static inline int max(int x, int y) { return x > y ? x : y; }

static int utf8_len(const uint8_t ch)
{
    int len = 0;
    for (utf_t **u = utf; (*u)->mask; ++u) {
        if ((ch & ~(*u)->mask) == (*u)->lead) {
            break;
        }
        ++len;
    }
    if (len > 4) { /* Malformed leading byte */
        assert("invalid unicode.");
    }
    return len;
}

static uint32_t next_cp(const uint8_t **string)
{
    if (**string == 0) {
        return 0;
    }
    int bytes = utf8_len(**string);
    const uint8_t *chr = *string;
    *string += bytes;
    int shift = utf[0]->bits_stored * (bytes - 1);
    uint32_t codep = (*chr++ & utf[bytes]->mask) << shift;

    for (int i = 1; i < bytes; ++i, ++chr) {
        shift -= utf[0]->bits_stored;
        codep |= ((const uint8_t) *chr & utf[0]->mask) << shift;
    }

    return codep;
}

EpdFontProperties epd_font_properties_default()
{
    EpdFontProperties props = {
        .fg_color = 0, .bg_color = 15, .fallback_glyph = 0, .flags = EPD_DRAW_ALIGN_LEFT
    };
    return props;
}

const EpdGlyph *epd_get_glyph(const EpdFont *font, uint32_t code_point)
{
    const EpdUnicodeInterval *intervals = font->intervals;
    for (unsigned int i = 0; i < font->interval_count; i++) {
        const EpdUnicodeInterval *interval = &intervals[i];
        if (code_point >= interval->first && code_point <= interval->last) {
            return &font->glyph[interval->offset + (code_point - interval->first)];
        }
        if (code_point < interval->first) {
            return NULL;
        }
    }
    return NULL;
}

#ifdef WITH_ZLIB
static int do_uncompress(uint8_t *dest, size_t uncompressed_size, const uint8_t *source, size_t source_size)
{
    if (uncompressed_size == 0 || dest == NULL || source_size == 0 || source == NULL) {
        return -1;
    }

    z_stream infstream;
    infstream.zalloc = Z_NULL;
    infstream.zfree = Z_NULL;
    infstream.opaque = Z_NULL;
    infstream.avail_in = (uInt) source_size;
    infstream.next_in = (void *) source;
    infstream.avail_out = uncompressed_size;
    infstream.next_out = dest;

    int ret = inflateInit(&infstream);
    if (ret != Z_OK) {
        fprintf(stderr, "Failed inflateInit\n");
        return -1;
    }
    ret = inflate(&infstream, Z_STREAM_END);
    if (ret != Z_STREAM_END) {
        fprintf(stderr, "Failed inflate: %i\n", ret);
        return -1;
    }
    inflateEnd(&infstream);

    return 0;
}
#else
static int do_uncompress(uint8_t *dest, size_t uncompressed_size, const uint8_t *source, size_t source_size)
{
    static tinfl_decompressor decomp;

    if (uncompressed_size == 0 || dest == NULL || source_size == 0 || source == NULL) {
        return -1;
    }
    tinfl_init(&decomp);

    // we know everything will fit into the buffer.
    tinfl_status decomp_status = tinfl_decompress(&decomp, source, &source_size, dest, dest, &uncompressed_size, TINFL_FLAG_PARSE_ZLIB_HEADER | TINFL_FLAG_USING_NON_WRAPPING_OUTPUT_BUF);
    if (decomp_status != TINFL_STATUS_DONE) {
        return decomp_status;
    }
    return 0;
}
#endif

/*!
   @brief   Draw a single character to a pre-allocated buffer.
*/
static enum EpdDrawError draw_char(const EpdFont *font, void *buffer,
    int *cursor_x, int cursor_y, uint32_t cp,
    const EpdFontProperties *props)
{

    assert(props != NULL);

    const EpdGlyph *glyph = epd_get_glyph(font, cp);
    if (!glyph) {
        glyph = epd_get_glyph(font, props->fallback_glyph);
    }

    if (!glyph) {
        return EPD_DRAW_GLYPH_FALLBACK_FAILED;
    }

    uint32_t offset = glyph->data_offset;
    uint16_t width = glyph->width, height = glyph->height;
    int left = glyph->left;

    int byte_width = (width / 2 + width % 2);
    unsigned long bitmap_size = byte_width * height;
    const uint8_t *bitmap = NULL;
    if (font->compressed) {
        uint8_t *tmp_bitmap = (uint8_t *) malloc(bitmap_size);
        if (tmp_bitmap == NULL && bitmap_size) {
            fprintf(stderr, "malloc failed.");
            return EPD_DRAW_FAILED_ALLOC;
        }
        do_uncompress(tmp_bitmap, bitmap_size, &font->bitmap[offset],
            glyph->compressed_size);
        bitmap = tmp_bitmap;
    } else {
        bitmap = &font->bitmap[offset];
    }

    uint8_t color_lut[16];
    for (int c = 0; c < 16; c++) {
        int color_difference = (int) props->fg_color - (int) props->bg_color;
        color_lut[c] = max(0, min(15, props->bg_color + c * color_difference / 15));
    }
    bool background_needed = props->flags & EPD_DRAW_BACKGROUND;

    for (int y = 0; y < height; y++) {
        int yy = cursor_y - glyph->top + y;
        int start_pos = *cursor_x + left;
        bool byte_complete = start_pos % 2;
        int x = max(0, -start_pos);
        int max_x = start_pos + width;
        uint8_t color;

        for (int xx = start_pos; xx < max_x; xx++) {
            uint8_t bm = bitmap[y * byte_width + x / 2];
            if ((x & 1) == 0) {
                bm = bm & 0xF;
            } else {
                bm = bm >> 4;
            }
            if (background_needed || bm) {
                color = color_lut[bm] << 4;
                epd_draw_pixel(xx, yy, color, buffer);
            }
            byte_complete = !byte_complete;
            x++;
        }
    }
    if (font->compressed) {
        free((uint8_t *) bitmap);
    }
    *cursor_x += glyph->advance_x;
    return EPD_DRAW_SUCCESS;
}

/*!
 * @brief Calculate the bounds of a character when drawn at (x, y), move the
 * cursor (*x) forward, adjust the given bounds.
 */
static void get_char_bounds(const EpdFont *font, uint32_t cp, int *x, int *y,
    int *minx, int *miny, int *maxx, int *maxy,
    const EpdFontProperties *props)
{

    assert(props != NULL);

    const EpdGlyph *glyph = epd_get_glyph(font, cp);

    if (!glyph) {
        glyph = epd_get_glyph(font, props->fallback_glyph);
    }

    if (!glyph) {
        return;
    }

    int x1 = *x + glyph->left, y1 = *y + glyph->top - glyph->height,
        x2 = x1 + glyph->width, y2 = y1 + glyph->height;

    // background needs to be taken into account
    if (props->flags & EPD_DRAW_BACKGROUND) {
        *minx = min(*x, min(*minx, x1));
        *maxx = max(max(*x + glyph->advance_x, x2), *maxx);
        *miny = min(*y + font->descender, min(*miny, y1));
        *maxy = max(*y + font->ascender, max(*maxy, y2));
    } else {
        if (x1 < *minx)
            *minx = x1;
        if (y1 < *miny)
            *miny = y1;
        if (x2 > *maxx)
            *maxx = x2;
        if (y2 > *maxy)
            *maxy = y2;
    }
    *x += glyph->advance_x;
}

EpdRect epd_get_string_rect(const EpdFont *font, const char *string,
    int x, int y, int margin, const EpdFontProperties *properties)
{
    assert(properties != NULL);
    EpdFontProperties props = *properties;
    props.flags |= EPD_DRAW_BACKGROUND;
    EpdRect temp = { .x = x, .y = y, .width = 0, .height = 0 };
    if (*string == '\0')
        return temp;
    int minx = 100000, miny = 100000, maxx = -1, maxy = -1;
    int temp_x = x;
    int temp_y = y + font->ascender;

    // Go through each line and get it's co-ordinates
    uint32_t c;
    while ((c = next_cp((const uint8_t **) &string))) {
        if (c == 0x000A) // newline
        {
            temp_x = x;
            temp_y += font->advance_y;
        } else
            get_char_bounds(font, c, &temp_x, &temp_y, &minx, &miny, &maxx, &maxy, &props);
    }
    temp.width = maxx - x + (margin * 2);
    temp.height = maxy - miny + (margin * 2);
    return temp;
}

void epd_get_text_bounds(const EpdFont *font, const char *string,
    const int *x, const int *y,
    int *x1, int *y1, int *w, int *h,
    const EpdFontProperties *properties)
{
    // FIXME: Does not respect alignment!

    assert(properties != NULL);
    EpdFontProperties props = *properties;

    if (*string == '\0') {
        *w = 0;
        *h = 0;
        *y1 = *y;
        *x1 = *x;
        return;
    }
    int minx = 100000, miny = 100000, maxx = -1, maxy = -1;
    int original_x = *x;
    int temp_x = *x;
    int temp_y = *y;
    uint32_t c;
    while ((c = next_cp((const uint8_t **) &string))) {
        get_char_bounds(font, c, &temp_x, &temp_y, &minx, &miny, &maxx, &maxy, &props);
    }
    *x1 = min(original_x, minx);
    *w = maxx - *x1;
    *y1 = miny;
    *h = maxy - miny;
}

static enum EpdDrawError epd_write_line(
    const EpdFont *font, const char *string, int *cursor_x,
    int *cursor_y, void *framebuffer,
    const EpdFontProperties *properties)
{

    assert(framebuffer != NULL);

    if (*string == '\0') {
        return EPD_DRAW_SUCCESS;
    }

    assert(properties != NULL);
    EpdFontProperties props = *properties;
    enum EpdFontFlags alignment_mask = EPD_DRAW_ALIGN_LEFT | EPD_DRAW_ALIGN_RIGHT | EPD_DRAW_ALIGN_CENTER;
    enum EpdFontFlags alignment = props.flags & alignment_mask;

    // alignments are mutually exclusive!
    if ((alignment & (alignment - 1)) != 0) {
        return EPD_DRAW_INVALID_FONT_FLAGS;
    }

    int x1 = 0, y1 = 0, w = 0, h = 0;
    int tmp_cur_x = *cursor_x;
    int tmp_cur_y = *cursor_y;
    epd_get_text_bounds(font, string, &tmp_cur_x, &tmp_cur_y, &x1, &y1, &w, &h, &props);

    // no printable characters
    if (w < 0 || h < 0) {
        return EPD_DRAW_NO_DRAWABLE_CHARACTERS;
    }

    int local_cursor_x = *cursor_x;
    int local_cursor_y = *cursor_y;
    uint32_t c;

    int cursor_x_init = local_cursor_x;
    int cursor_y_init = local_cursor_y;

    switch (alignment) {
        case EPD_DRAW_ALIGN_LEFT: {
            break;
        }
        case EPD_DRAW_ALIGN_CENTER: {
            local_cursor_x -= w / 2;
            break;
        }
        case EPD_DRAW_ALIGN_RIGHT: {
            local_cursor_x -= w;
            break;
        }
        default:
            break;
    }

    //FIXME: needed from epd_draw_hline
    //uint8_t bg = props.bg_color;
    if (props.flags & EPD_DRAW_BACKGROUND) {
        for (int l = local_cursor_y - font->ascender;
             l < local_cursor_y - font->descender; l++) {
            //FIXME: following function is not implemented
            //epd_draw_hline(local_cursor_x, l, w, bg << 4, framebuffer);
        }
    }
    enum EpdDrawError err = EPD_DRAW_SUCCESS;
    while ((c = next_cp((const uint8_t **) &string))) {
        err |= draw_char(font, framebuffer, &local_cursor_x, local_cursor_y, c, &props);
    }

    *cursor_x += local_cursor_x - cursor_x_init;
    *cursor_y += local_cursor_y - cursor_y_init;
    return err;
}

enum EpdDrawError epd_write_default(const EpdFont *font, const char *string, int *cursor_x,
    int *cursor_y, void *framebuffer)
{
    const EpdFontProperties props = epd_font_properties_default();
    return epd_write_string(font, string, cursor_x, cursor_y, framebuffer, &props);
}

enum EpdDrawError epd_write_string(
    const EpdFont *font, const char *string, int *cursor_x,
    int *cursor_y, void *framebuffer,
    const EpdFontProperties *properties)
{
    char *token, *newstring, *tofree;
    if (string == NULL) {
        fprintf(stderr, "cannot draw a NULL string!");
        return EPD_DRAW_STRING_INVALID;
    }
    tofree = newstring = strdup(string);
    if (newstring == NULL) {
        fprintf(stderr, "cannot allocate string copy!");
        return EPD_DRAW_FAILED_ALLOC;
    }

    enum EpdDrawError err = EPD_DRAW_SUCCESS;
    // taken from the strsep manpage
    int line_start = *cursor_x;
    while ((token = strsep(&newstring, "\n")) != NULL) {
        *cursor_x = line_start;
        err |= epd_write_line(font, token, cursor_x, cursor_y, framebuffer, properties);
        *cursor_y += font->advance_y;
    }

    free(tofree);
    return err;
}

EpdFont *ufont_load_font(const void *ufont, const void *glyph, const void *intervals, const void *bitmap)
{
    struct __attribute__((__packed__))
    {
        uint32_t interval_count;
        uint8_t compressed;
        uint16_t advance_y;
        uint16_t ascender;
        uint16_t descender;
    } serialized_ufont;

    memcpy(&serialized_ufont, ufont, sizeof(serialized_ufont));

    EpdFont *loaded_font = malloc(sizeof(EpdFont));
    loaded_font->bitmap = bitmap;
    loaded_font->glyph = glyph;
    loaded_font->intervals = intervals;
    loaded_font->interval_count = serialized_ufont.interval_count;
    loaded_font->compressed = serialized_ufont.compressed;
    loaded_font->advance_y = serialized_ufont.advance_y;
    loaded_font->ascender = serialized_ufont.ascender;
    loaded_font->descender = serialized_ufont.descender;

    return loaded_font;
}

#define GET_LIST_ENTRY(list_item, type, list_head_member) \
    ((type *) (((char *) (list_item)) - ((unsigned long) &((type *) 0)->list_head_member)))

#define LIST_FOR_EACH(item, head) \
    for (item = (head)->next; item != (head); item = item->next)

#define MUTABLE_LIST_FOR_EACH(item, tmp, head) \
    for (item = (head)->next, tmp = item->next; item != (head); item = tmp, tmp = item->next)

struct UFListHead;

struct UFListHead
{
    struct UFListHead *next;
    struct UFListHead *prev;
};

static inline void uflist_insert(struct UFListHead *new_item, struct UFListHead *prev_head, struct UFListHead *next_head)
{
    new_item->prev = prev_head;
    new_item->next = next_head;
    next_head->prev = new_item;
    prev_head->next = new_item;
}

static inline void uflist_append(struct UFListHead *head, struct UFListHead *new_item)
{
    uflist_insert(new_item, head->prev, head);
}

static inline void uflist_remove(struct UFListHead *remove_item)
{
    remove_item->prev->next = remove_item->next;
    remove_item->next->prev = remove_item->prev;
}

static inline void uflist_init(struct UFListHead *list_item)
{
    list_item->prev = list_item;
    list_item->next = list_item;
}

typedef struct
{
    struct UFListHead list_head;
    const char *handle;
    EpdFont *font;
} UFont;

struct UFontManager
{
    struct UFListHead font_list;
};

UFontManager *ufont_manager_new()
{
    UFontManager *ufont_manager = malloc(sizeof(UFontManager));
    uflist_init(&ufont_manager->font_list);

    return ufont_manager;
}

void ufont_manager_register(UFontManager *ufont_manager, const char *handle, EpdFont *font)
{
    UFont *ufont = malloc(sizeof(UFont));
    ufont->handle = strdup(handle);
    ufont->font = font;
    uflist_append(&ufont_manager->font_list, &ufont->list_head);
}

EpdFont *ufont_manager_find_by_handle(UFontManager *ufont_manager, const char *handle)
{
    struct UFListHead *item;
    LIST_FOR_EACH (item, &ufont_manager->font_list) {
        UFont *ufont = GET_LIST_ENTRY(item, UFont, list_head);
        if (!strcmp(handle, ufont->handle)) {
            return ufont->font;
        }
    }

    return NULL;
}

#ifdef __ORDER_LITTLE_ENDIAN__
    #ifdef __GNUC__
        #define UF_ENDIAN_SWAP_32(value) __builtin_bswap32(value)
    #else
        #define UF_ENDIAN_SWAP_32(value) ((((value) & 0xFF) << 24) | (((value) & 0xFF00) << 8) | \
                (((value) & 0xFF0000) >> 8) | (((value) & 0xFF000000) >> 24))
    #endif
#else
    #define UF_ENDIAN_SWAP_32(value) (value)
#endif

struct UFIFFRecord
{
    const char name[4];
    uint32_t size;
};

static uint32_t ufont_iff_align(uint32_t size)
{
    return ((size + 4 - 1) >> 2) << 2;
}

static int ufont_iff_is_valid_ufl(const void *iff)
{
    return memcmp(iff, "UFL0", 4) == 0;
}

EpdFont *ufont_parse(const void *iff_binary, int buf_size)
{
    if (ufont_iff_is_valid_ufl(iff_binary)) {
        return NULL;
    }

    const uint8_t *data = iff_binary;

    int current_pos = 12;

    uint32_t iff_size = UF_ENDIAN_SWAP_32(*(((uint32_t *) (data + 4))));
    int file_size = iff_size;
    if (buf_size < file_size) {
        fprintf(stderr, "warning: buffer holding IFF %i is smaller than IFF size: %i", buf_size, file_size);
        return NULL;
    }

    const void *ufont = NULL;
    const void *glyph = NULL;
    const void *intervals = NULL;
    const void *bitmap = NULL;

    do {
        struct UFIFFRecord *current_record = (struct UFIFFRecord *) (data + current_pos);

        if (!memcmp(current_record->name, "uFH0", 4)) {
            ufont = data + current_pos + sizeof(struct UFIFFRecord);

        } else if (!memcmp(current_record->name, "uFP0", 4)) {
            glyph = data + current_pos + sizeof(struct UFIFFRecord);

        } else if (!memcmp(current_record->name, "uFI0", 4)) {
            intervals = data + current_pos + sizeof(struct UFIFFRecord);

        } else if (!memcmp(current_record->name, "uFB0", 4)) {
            bitmap = data + current_pos + sizeof(struct UFIFFRecord);
        }

        current_pos += ufont_iff_align(UF_ENDIAN_SWAP_32(current_record->size) + 8);
    } while (current_pos < file_size);

    return ufont_load_font(ufont, glyph, intervals, bitmap);
}
