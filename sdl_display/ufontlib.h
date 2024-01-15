#ifdef __cplusplus
extern "C" {
#endif

#pragma once
#include <stdbool.h>
#include <stdint.h>

/// Font data stored PER GLYPH
typedef struct __attribute__((__packed__)) {
  uint16_t width;            ///< Bitmap dimensions in pixels
  uint16_t height;           ///< Bitmap dimensions in pixels
  uint16_t advance_x;        ///< Distance to advance cursor (x axis)
  int16_t left;             ///< X dist from cursor pos to UL corner
  int16_t top;              ///< Y dist from cursor pos to UL corner
  uint32_t compressed_size; ///< Size of the zlib-compressed font data.
  uint32_t data_offset;     ///< Pointer into EpdFont->bitmap
} EpdGlyph;

/// Glyph interval structure
typedef struct __attribute__((__packed__))  {
  uint32_t first;  ///< The first unicode code point of the interval
  uint32_t last;   ///< The last unicode code point of the interval
  uint32_t offset; ///< Index of the first code point into the glyph array
} EpdUnicodeInterval;

/// Data stored for FONT AS A WHOLE
typedef struct {
  const uint8_t *bitmap;            ///< Glyph bitmaps, concatenated
  const EpdGlyph *glyph;            ///< Glyph array
  const EpdUnicodeInterval *intervals; ///< Valid unicode intervals for this font
  uint32_t interval_count;    ///< Number of unicode intervals.
  bool compressed;            ///< Does this font use compressed glyph bitmaps?
  uint16_t advance_y;         ///< Newline distance (y axis)
  int ascender;               ///< Maximal height of a glyph above the base line
  int descender;              ///< Maximal height of a glyph below the base line
} EpdFont;

/// An area on the display.
typedef struct {
  /// Horizontal position.
  int x;
  /// Vertical position.
  int y;
  /// Area / image width, must be positive.
  int width;
  /// Area / image height, must be positive.
  int height;
} EpdRect;

/// Possible failures when drawing.
enum EpdDrawError {
  EPD_DRAW_SUCCESS = 0x0,
  /// The string to draw is invalid.
  EPD_DRAW_STRING_INVALID = 0x4,

  /// The string was not empty, but no characters where drawable.
  EPD_DRAW_NO_DRAWABLE_CHARACTERS = 0x8,

  /// Allocation failed
  EPD_DRAW_FAILED_ALLOC = 0x10,

  /// A glyph could not be drawn, and not fallback was present.
  EPD_DRAW_GLYPH_FALLBACK_FAILED = 0x20,

  /// An invalid combination of font flags was used.
  EPD_DRAW_INVALID_FONT_FLAGS = 0x200,
};

/// Font drawing flags
enum EpdFontFlags {
  /// Draw a background.
  ///
  /// Take the background into account
  /// when calculating the size.
  EPD_DRAW_BACKGROUND = 0x1,

  /// Left-Align lines
  EPD_DRAW_ALIGN_LEFT = 0x2,
  /// Right-align lines
  EPD_DRAW_ALIGN_RIGHT = 0x4,
  /// Center-align lines
  EPD_DRAW_ALIGN_CENTER = 0x8,
};

/// Font properties.
typedef struct {
  /// Foreground color
  uint8_t fg_color : 4;
  /// Background color
  uint8_t bg_color : 4;
  /// Use the glyph for this codepoint for missing glyphs.
  uint32_t fallback_glyph;
  /// Additional flags, reserved for future use
  enum EpdFontFlags flags;
} EpdFontProperties;

/**
 * Draw a pixel a given framebuffer.
 *
 * @param x: Horizontal position in pixels.
 * @param y: Vertical position in pixels.
 * @param color: The gray value of the line (see [Colors](#Colors));
 * @param framebuffer: The framebuffer to draw to,
 */
void epd_draw_pixel(int x, int y, uint8_t color, void *framebuffer);

/**
 * Draw a horizontal line to a given framebuffer.
 *
 * @param x: Horizontal start position in pixels.
 * @param y: Vertical start position in pixels.
 * @param length: Length of the line in pixels.
 * @param color: The gray value of the line (see [Colors](#Colors));
 * @param framebuffer: The framebuffer to draw to,
 *  which must be `EPD_WIDTH / 2 * EPD_HEIGHT` bytes large.
 */
void epd_draw_hline(int x, int y, int length, uint8_t color,
                    void *framebuffer);

/**
 * The default font properties.
 */
EpdFontProperties epd_font_properties_default();

/*!
 * Get the text bounds for string, when drawn at (x, y).
 * Set font properties to NULL to use the defaults.
 */
void epd_get_text_bounds(const EpdFont *font, const char *string,
                     const int *x, const int *y,
                     int *x1, int *y1, int *w, int *h,
                     const EpdFontProperties *props);
/*!
 * Returns a rect with the bounds of the text
 * @param font : the font used to get the character sizes
 * @param string: pointer to c string
 * @param x : left most position of rectangle
 * @param y : top most point of the rectangle
 * @param margin : to be pllied to the width and height
 * @returns EpdRect with x and y as per the original and height and width
 *       adjusted to fit the text with the margin added as well.
 */ 
EpdRect epd_get_string_rect (const EpdFont *font, const char *string,
                     int x, int y, int margin, const EpdFontProperties *properties );

/**
 * Write text to the EPD.
 */
enum EpdDrawError epd_write_string(const EpdFont *font, const char *string, int *cursor_x,
                int *cursor_y, void *framebuffer,
                const EpdFontProperties *properties);

/**
 * Write a (multi-line) string to the EPD.
 */
enum EpdDrawError epd_write_default(const EpdFont *font, const char *string, int *cursor_x,
                  int *cursor_y, void *framebuffer);

/**
 * Get the font glyph for a unicode code point.
 */
const EpdGlyph* epd_get_glyph(const EpdFont *font, uint32_t code_point);

EpdFont *ufont_load_font(const void *ufont, const void *glyph, const void *intervals,
        const void *bitmap);

struct UFontManager;
typedef struct UFontManager UFontManager;

UFontManager *ufont_manager_new();
void ufont_manager_register(UFontManager *ufont_manager, const char *handle, EpdFont *font);
EpdFont *ufont_manager_find_by_handle(UFontManager *ufont_manager, const char *handle);

EpdFont *ufont_parse(const void *iff_binary, int buf_size);

#ifdef __cplusplus
}
#endif
