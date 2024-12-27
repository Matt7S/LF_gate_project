#ifndef FONTS_HPP
#define FONTS_HPP

#include <Arduino.h>

// extern Numbers[];
typedef struct {
	uint16_t FontWidth;    /*!< Font width in pixels */
	uint16_t FontHeight;   /*!< Font height in pixels */
	const uint8_t *data; /*!< Pointer to data font data array */
} FontDef_t;

/**
 * @brief  String length and height
 */
typedef struct {
	uint16_t Length;      /*!< String length in units of pixels */
	uint16_t Height;      /*!< String height in units of pixels */
} FONTS_SIZE_t;

extern FontDef_t Numbers_6x8; 		// font_arial_14pt.c
extern FontDef_t Arial10_c_5x8;////////////////
typedef struct {
     const uint8_t *data;
     uint16_t width;
     uint16_t height;
     uint8_t dataSize;
     } tImage;
typedef struct {
    long int code;
    const tImage *image;
} tChar;
typedef struct {
    int length;
    const tChar *chars;
} tFont;

extern const tFont Arial10_c;


#endif // FONTS_HPP
