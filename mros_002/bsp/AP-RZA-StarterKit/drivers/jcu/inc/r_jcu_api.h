/*******************************************************************************
* DISCLAIMER
* This software is supplied by Renesas Electronics Corporation and is only
* intended for use with Renesas products. No other uses are authorized. This
* software is owned by Renesas Electronics Corporation and is protected under
* all applicable laws, including copyright laws.
* THIS SOFTWARE IS PROVIDED "AS IS" AND RENESAS MAKES NO WARRANTIES REGARDING
* THIS SOFTWARE, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING BUT NOT
* LIMITED TO WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE
* AND NON-INFRINGEMENT. ALL SUCH WARRANTIES ARE EXPRESSLY DISCLAIMED.
* TO THE MAXIMUM EXTENT PERMITTED NOT PROHIBITED BY LAW, NEITHER RENESAS
* ELECTRONICS CORPORATION NOR ANY OF ITS AFFILIATED COMPANIES SHALL BE LIABLE
* FOR ANY DIRECT, INDIRECT, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR
* ANY REASON RELATED TO THIS SOFTWARE, EVEN IF RENESAS OR ITS AFFILIATES HAVE
* BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
* Renesas reserves the right, without notice, to make changes to this software
* and to discontinue the availability of this software. By using this software,
* you agree to the additional terms and conditions found by accessing the
* following link:
* http://www.renesas.com/disclaimer
* Copyright (C) 2012 - 2016 Renesas Electronics Corporation. All rights reserved.
*******************************************************************************/
/******************************************************************************
* File: r_jcu_api.h
*    JPEG Codec Unit (JCU) driver's API. Main Header.
*
* - $Module: JCU $ $PublicVersion: 1.03 $ (=JCU_VERSION)
* - $Rev: 38 $
* - $Date:: 2014-03-18 16:14:45 +0900#$
******************************************************************************/

#ifndef JCU_API_H_
#define JCU_API_H_

/******************************************************************************
Includes   <System Includes> , "Project Includes"
******************************************************************************/
#include    <stddef.h>
#include    "r_jcu_user.h"
#include    "r_ospl.h"

/******************************************************************************
Typedef definitions
******************************************************************************/

/***********************************************************************
* Enumeration: jcu_status_information_t
*    Status
*
*    : JCU_STATUS_UNDEF        - 0x00, Undefined
*    : JCU_STATUS_INIT         - 0x01, Initialized
*    : JCU_STATUS_SELECTED     - 0x02, Codec is selected
*    : JCU_STATUS_READY        - 0x08, Ready
*    : JCU_STATUS_RUN          - 0x10, Run
*    : JCU_STATUS_INTERRUPTING - 0x40, Interrupting
*    : JCU_STATUS_INTERRUPTED  - 0x80, Interrupted
************************************************************************/
typedef enum
{
    JCU_STATUS_UNDEF        = 0x00,
    JCU_STATUS_INIT         = 0x01,
    JCU_STATUS_SELECTED     = 0x02,
    JCU_STATUS_READY        = 0x08,
    JCU_STATUS_RUN          = 0x10,
    JCU_STATUS_INTERRUPTING = 0x40,
    JCU_STATUS_INTERRUPTED  = 0x80
} jcu_status_information_t;


/***********************************************************************
* Structure: jcu_async_status_t
*    Status of asynchronize
************************************************************************/
typedef struct st_jcu_async_status_t  jcu_async_status_t;
struct st_jcu_async_status_t
{
    jcu_status_information_t  Status;
    bit_flags_fast32_t        SubStatusFlags;
    bool_t                    IsPaused;
    bool_t                    IsEnabledInterrupt;
    r_ospl_flag32_t           InterruptEnables;
    r_ospl_flag32_t           InterruptFlags;
    r_ospl_flag32_t           CancelFlags;  /*<r_ospl_cancel_flag_t>*/
};


/* Section: Global */
/***********************************************************************
* Enumeration: jcu_sub_status_t
*    Sub Status
*
*    : JCU_SUB_INFOMATION_READY    - 0x00000008
*    : JCU_SUB_DECODE_OUTPUT_PAUSE - 0x00000100
*    : JCU_SUB_DECODE_INPUT_PAUSE  - 0x00000200
*    : JCU_SUB_ENCODE_OUTPUT_PAUSE - 0x00001000
*    : JCU_SUB_ENCODE_INPUT_PAUSE  - 0x00002000
************************************************************************/
typedef enum
{
    JCU_SUB_INFOMATION_READY    = 0x00000008,
    JCU_SUB_DECODE_OUTPUT_PAUSE = 0x00000100,
    JCU_SUB_DECODE_INPUT_PAUSE  = 0x00000200,
    JCU_SUB_ENCODE_OUTPUT_PAUSE = 0x00001000,
    JCU_SUB_ENCODE_INPUT_PAUSE  = 0x00002000,
    JCU_SUB_PAUSE_ALL           = JCU_SUB_INFOMATION_READY | JCU_SUB_DECODE_OUTPUT_PAUSE |
                                  JCU_SUB_DECODE_INPUT_PAUSE | JCU_SUB_ENCODE_OUTPUT_PAUSE | JCU_SUB_ENCODE_INPUT_PAUSE
} jcu_sub_state_t;
typedef bit_flags_fast32_t jcu_sub_status_t;


/***********************************************************************
* Constants: jcu_interrupt_line_t
*    Interrupt Lines
*
*    : JCU_INTERRUPT_LINE_JEDI - 1
*    : JCU_INTERRUPT_LINE_JDTI - 2
*    : JCU_INTERRUPT_LINE_ALL  - 1 | 2
************************************************************************/
typedef enum
{
    JCU_INTERRUPT_LINE_JEDI  = 0x00000001u,
    JCU_INTERRUPT_LINE_JDTI  = 0x00000002u,
    JCU_INTERRUPT_LINE_ALL   = ( JCU_INTERRUPT_LINE_JEDI | JCU_INTERRUPT_LINE_JDTI )
} jcu_interrupt_line_t;


/***********************************************************************
* Type: jcu_interrupt_lines_t
*    Bit flags of <jcu_interrupt_line_t>
************************************************************************/
typedef bit_flags_fast32_t /*<jcu_interrupt_line_t>*/  jcu_interrupt_lines_t;


/******************************************************************************
Macro definitions
******************************************************************************/

/***********************************************************************
* Macro: JCU_VERSION
*    VERSION
************************************************************************/
#define JCU_VERSION  103


/***********************************************************************
* Macro: JCU_VERSION_STRING
*    String of <JCU_VERSION>
************************************************************************/
#define JCU_VERSION_STRING  "1.03"


/***********************************************************************
* Macro: JCU_INT_TYPE_NUM
*    The number of the interrupt source
************************************************************************/
#define JCU_INT_TYPE_NUM  0x0009


/***********************************************************************
* Macro: JCU_COLOR_ELEMENT_NUM
*    Y,Cb and Cr
************************************************************************/
#define JCU_COLOR_ELEMENT_NUM   3


/******************************************************************************
Struct & Enum definitions
******************************************************************************/

/***********************************************************************
* Enumeration: jcu_errorcode_t
*    Error code of JCU driver
*
*    : JCU_ERROR_OK         - 0
*    : JCU_ERROR_PARAM      - 0x4501
*    : JCU_ERROR_STATUS     - 0x4502
*    : JCU_ERROR_CODEC_TYPE - 0x4503
*    : JCU_ERROR_LIMITATION - 0x4504
************************************************************************/
enum { E_CATEGORY_JCU_ERR = 0x4500 };  /* Offset of JCU error code */
typedef errnum_t  jcu_errorcode_t;
enum
{
    JCU_ERROR_OK            = 0x0000,
    JCU_ERROR_PARAM         = 0x4501,
    JCU_ERROR_STATUS        = 0x4502,
    JCU_ERROR_CODEC_TYPE    = 0x4503,
    JCU_ERROR_LIMITATION    = 0x4504
};


/***********************************************************************
* Enumeration: jcu_detail_error_t
*    Kind of detailed error for decoding
*
*    : JCU_JCDERR_OK                - 0x0000
*    : JCU_JCDERR_SOI_NOT_FOUND     - 0x4521
*    : JCU_JCDERR_INVALID_SOF       - 0x4522
*    : JCU_JCDERR_UNPROVIDED_SOF    - 0x4523
*    : JCU_JCDERR_SOF_ACCURACY      - 0x4524
*    : JCU_JCDERR_DQT_ACCURACY      - 0x4525
*    : JCU_JCDERR_COMPONENT_1       - 0x4526
*    : JCU_JCDERR_COMPONENT_2       - 0x4527
*    : JCU_JCDERR_NO_SOF0_DQT_DHT   - 0x4528
*    : JCU_JCDERR_SOS_NOT_FOUND     - 0x4529
*    : JCU_JCDERR_EOI_NOT_FOUND     - 0x452A
*    : JCU_JCDERR_RESTART_INTERVAL  - 0x452B
*    : JCU_JCDERR_IMAGE_SIZE        - 0x452C
*    : JCU_JCDERR_LAST_MCU_DATA     - 0x452D
*    : JCU_JCDERR_BLOCK_DATA        - 0x452E
************************************************************************/
enum { E_CATEGORY_JCU_JCDERR = 0x4520 };  /* Offset of JCU detail error code */
typedef errnum_t  jcu_detail_error_t;
enum
{
    JCU_JCDERR_OK                = 0x0000,
    JCU_JCDERR_SOI_NOT_FOUND     = 0x4521,
    JCU_JCDERR_INVALID_SOF       = 0x4522,
    JCU_JCDERR_UNPROVIDED_SOF    = 0x4523,
    JCU_JCDERR_SOF_ACCURACY      = 0x4524,
    JCU_JCDERR_DQT_ACCURACY      = 0x4525,
    JCU_JCDERR_COMPONENT_1       = 0x4526,
    JCU_JCDERR_COMPONENT_2       = 0x4527,
    JCU_JCDERR_NO_SOF0_DQT_DHT   = 0x4528,
    JCU_JCDERR_SOS_NOT_FOUND     = 0x4529,
    JCU_JCDERR_EOI_NOT_FOUND     = 0x452A,
    JCU_JCDERR_RESTART_INTERVAL  = 0x452B,
    JCU_JCDERR_IMAGE_SIZE        = 0x452C,
    JCU_JCDERR_LAST_MCU_DATA     = 0x452D,
    JCU_JCDERR_BLOCK_DATA        = 0x452E
};


/***********************************************************************
* Enumeration: jcu_codec_t
*    The type of codec
*
*    : JCU_ENCODE - 0
*    : JCU_DECODE - 1
************************************************************************/
typedef enum
{
    JCU_ENCODE  = 0,
    JCU_DECODE  = 1
} jcu_codec_t;


/***********************************************************************
* Structure: jcu_count_mode_param_t
*    Buffer settings for separating decode or encode
************************************************************************/
typedef struct
{
    struct
    {
        /* Variable: isEnable
            true: It uses the count mode process. */
        bool_t      isEnable;

        /* Variable: isInitAddress
            Address is changed when processing is stopped. */
        bool_t      isInitAddress;

        /* Variable: restartAddress
            This is enable when isInitAddrss is true. */
        uint32_t   *restartAddress;

        /* Variable: dataCount
            Encode:8line Decode:8byte. */
        uint32_t    dataCount;

    } inputBuffer;

    struct
    {
        /* Variable: isEnable
            true: It uses the count mode process. */
        bool_t      isEnable;

        /* Variable: isInitAddress
            Address is changed when processing is stopped. */
        bool_t      isInitAddress;      /*    */

        /* Variable: restartAddress
            This is enable when isInitAddress is true. */
        uint32_t    *restartAddress;    /*     */

        /* Variable: dataCount
            Encode:8byte Decode:8line. */
        uint32_t    dataCount;

    } outputBuffer;
} jcu_count_mode_param_t;


/* Section: Global */
/***********************************************************************
* Enumeration: jcu_continue_type_t
*    Type of continue buffer
*
*    : JCU_INPUT_BUFFER  - 0
*    : JCU_OUTPUT_BUFFER - 1
*    : JCU_IMAGE_INFO    - 2
************************************************************************/
typedef enum
{
    JCU_INPUT_BUFFER,
    JCU_OUTPUT_BUFFER,
    JCU_IMAGE_INFO
} jcu_continue_type_t;


/***********************************************************************
* Constants: jcu_int_detail_error_t
*    Bit flags of detail error
*
*    JCU_INT_ERROR_RESTART_INTERVAL_DATA - INT7_MASK
*    JCU_INT_ERROR_SEGMENT_TOTAL_DATA    - INT6_MASK
*    JCU_INT_ERROR_MCU_BLOCK_DATA        - INT5_MASK
*    JCU_INT_ERROR_ALL                   - e.g. for mask
************************************************************************/
typedef enum
{
    JCU_INT_ERROR_RESTART_INTERVAL_DATA = 0x80u,
    JCU_INT_ERROR_SEGMENT_TOTAL_DATA    = 0x40u,
    JCU_INT_ERROR_MCU_BLOCK_DATA        = 0x20u,
    JCU_INT_ERROR_ALL                   = ( JCU_INT_ERROR_RESTART_INTERVAL_DATA |
            JCU_INT_ERROR_SEGMENT_TOTAL_DATA | JCU_INT_ERROR_MCU_BLOCK_DATA )
} jcu_int_detail_error_t;


/***********************************************************************
* Type: jcu_int_detail_errors_t
*    Bit flags of <jcu_int_detail_error_t>
************************************************************************/
typedef bit_flags_fast32_t  jcu_int_detail_errors_t;


/***********************************************************************
* Enumeration: jcu_swap_t
*    SampleEnum
*
*    : JCU_SWAP_NONE                        - 01234567
*    : JCU_SWAP_BYTE                        - 10325476
*    : JCU_SWAP_WORD                        - 23016745
*    : JCU_SWAP_WORD_AND_BYTE               - 32107654
*    : JCU_SWAP_LONG_WORD                   - 45670123
*    : JCU_SWAP_LONG_WORD_AND_BYTE          - 54761032
*    : JCU_SWAP_LONG_WORD_AND_WORD          - 67452301
*    : JCU_SWAP_LONG_WORD_AND_WORD_AND_BYTE - 76543210
************************************************************************/
typedef enum
{
    JCU_SWAP_NONE                        = 0x00,
    JCU_SWAP_BYTE                        = 0x01,
    JCU_SWAP_WORD                        = 0x02,
    JCU_SWAP_WORD_AND_BYTE               = 0x03,
    JCU_SWAP_LONG_WORD                   = 0x04,
    JCU_SWAP_LONG_WORD_AND_BYTE          = 0x05,
    JCU_SWAP_LONG_WORD_AND_WORD          = 0x06,
    JCU_SWAP_LONG_WORD_AND_WORD_AND_BYTE = 0x07
} jcu_swap_t;


/***********************************************************************
* Structure: jcu_buffer_t
*    Buffer settings
************************************************************************/
typedef struct
{
    jcu_swap_t    swapSetting;
    uint32_t     *address;
} jcu_buffer_t;


/***********************************************************************
* Structure: jcu_buffer_param_t
*    Buffer parameter for input and output
************************************************************************/
typedef struct
{
    jcu_buffer_t  source;
    jcu_buffer_t  destination;
    int16_t       lineOffset;
} jcu_buffer_param_t;


/* Section: Global */
/***********************************************************************
* Enumeration: jcu_sub_sampling_t
*    Sub sampling settings for decoding
*
*    : JCU_SUB_SAMPLING_1_1 - 0x00 = 1/1
*    : JCU_SUB_SAMPLING_1_2 - 0x01 = 1/2
*    : JCU_SUB_SAMPLING_1_4 - 0x02 = 1/4
*    : JCU_SUB_SAMPLING_1_8 - 0x03 = 1/8
************************************************************************/
typedef enum
{
    JCU_SUB_SAMPLING_1_1 = 0x00,
    JCU_SUB_SAMPLING_1_2 = 0x01,
    JCU_SUB_SAMPLING_1_4 = 0x02,
    JCU_SUB_SAMPLING_1_8 = 0x03
} jcu_sub_sampling_t;


/***********************************************************************
* Enumeration: jcu_decode_format_t
*    Kind of pixel format can process the decoding
*
*    : JCU_OUTPUT_YCbCr422 - 0x00
*    : JCU_OUTPUT_ARGB8888 - 0x01
*    : JCU_OUTPUT_RGB565   - 0x02
************************************************************************/
typedef enum
{
    JCU_OUTPUT_YCbCr422 = 0x00,
    JCU_OUTPUT_ARGB8888 = 0x01,
    JCU_OUTPUT_RGB565   = 0x02
} jcu_decode_format_t;


/***********************************************************************
* Enumeration: jcu_cbcr_offset_t
*    Cb/Cr offset
*
*    : JCU_CBCR_OFFSET_0   - 0 = No offset
*    : JCU_CBCR_OFFSET_128 - 1 = +128 offset
************************************************************************/
typedef enum
{
    JCU_CBCR_OFFSET_0 = 0,
    JCU_CBCR_OFFSET_128 = 1
} jcu_cbcr_offset_t;


/***********************************************************************
* Enumeration: jcu_jpeg_format_t
*    Kind of pixel format for the jpeg file format when decoding
*
*    : JCU_JPEG_YCbCr444 - 0
*    : JCU_JPEG_YCbCr422 - 1
*    : JCU_JPEG_YCbCr420 - 2
*    : JCU_JPEG_YCbCr411 - 6
************************************************************************/
typedef enum
{
    JCU_JPEG_YCbCr444   = 0x00,
    JCU_JPEG_YCbCr422   = 0x01,
    JCU_JPEG_YCbCr420   = 0x02,
    JCU_JPEG_YCbCr411   = 0x06
} jcu_jpeg_format_t;


/***********************************************************************
* Structure: jcu_image_info_t
*    Image information data store variable
************************************************************************/
typedef struct
{
    uint32_t            width;
    uint32_t            height;
    jcu_jpeg_format_t   encodedFormat;
} jcu_image_info_t;


/* Section: Global */
/***********************************************************************
* Enumeration: jcu_table_no_t
*    The index of the table
*
*    : JCU_TABLE_NO_0 - 0
*    : JCU_TABLE_NO_1 - 1
*    : JCU_TABLE_NO_2 - 2
*    : JCU_TABLE_NO_3 - 3
************************************************************************/
typedef enum
{
    JCU_TABLE_NO_0 = 0,
    JCU_TABLE_NO_1 = 1,
    JCU_TABLE_NO_2 = 2,
    JCU_TABLE_NO_3 = 3
} jcu_table_no_t;


/***********************************************************************
* Enumeration: jcu_huff_t
*    Kind of Huffman table
*
*    : JCU_HUFFMAN_AC - 0
*    : JCU_HUFFMAN_DC - 1
************************************************************************/
typedef enum
{
    JCU_HUFFMAN_AC,
    JCU_HUFFMAN_DC
} jcu_huff_t;


/***********************************************************************
* Enumeration: jcu_color_element_t
*    Kind of color data for using the encoder parameter
*
*    : JCU_ELEMENT_Y  - 0
*    : JCU_ELEMENT_Cb - 1
*    : JCU_ELEMENT_Cr - 2
************************************************************************/
typedef enum
{
    JCU_ELEMENT_Y,
    JCU_ELEMENT_Cb,
    JCU_ELEMENT_Cr
} jcu_color_element_t;


/***********************************************************************
* Structure: jcu_decode_param_t
*    Setting parameter for the Decoding
************************************************************************/
typedef struct
{
    jcu_sub_sampling_t  verticalSubSampling;
    jcu_sub_sampling_t  horizontalSubSampling;
    jcu_decode_format_t decodeFormat;
    jcu_cbcr_offset_t   outputCbCrOffset;
    uint8_t             alpha;
} jcu_decode_param_t;


/***********************************************************************
* Structure: jcu_encode_param_t
*    Setting parameter for the Encoding
************************************************************************/
typedef struct
{
    jcu_jpeg_format_t   encodeFormat;
    int32_t             QuantizationTable[JCU_COLOR_ELEMENT_NUM];
    int32_t             HuffmanTable[JCU_COLOR_ELEMENT_NUM];
    uint32_t            DRI_value;
    uint32_t            width;
    uint32_t            height;
#ifndef _SH
    jcu_cbcr_offset_t   inputCbCrOffset;
#endif
} jcu_encode_param_t;


/* Section: Global */
/***********************************************************************
* Enumeration: jcu_codec_status_t
*    Codec type
*
*    : JCU_CODEC_NOT_SELECTED - -1
*    : JCU_STATUS_ENCODE      - 0
*    : JCU_STATUS_DECODE      - 1
************************************************************************/
typedef enum
{
    JCU_CODEC_NOT_SELECTED  = -1,
    JCU_STATUS_ENCODE       = 0,
    JCU_STATUS_DECODE       = 1
} jcu_codec_status_t;


/******************************************************************************
Variable Externs
******************************************************************************/

/******************************************************************************
Imported global variables and functions (from other files)
******************************************************************************/

/******************************************************************************
Functions Prototypes
******************************************************************************/
jcu_errorcode_t R_JCU_Initialize( void* const  NullConfig );
jcu_errorcode_t R_JCU_Terminate(void);
jcu_errorcode_t R_JCU_TerminateAsync( r_ospl_async_t* const  async );
jcu_errorcode_t R_JCU_SelectCodec(const jcu_codec_t codec);
jcu_errorcode_t R_JCU_Start(void);
jcu_errorcode_t R_JCU_StartAsync( r_ospl_async_t* const  async );
jcu_errorcode_t R_JCU_SetCountMode(const jcu_count_mode_param_t *const buffer);
jcu_errorcode_t R_JCU_Continue(const jcu_continue_type_t type);
jcu_errorcode_t R_JCU_ContinueAsync(const jcu_continue_type_t type, r_ospl_async_t* const  async);
void            R_JCU_GetAsyncStatus(const jcu_async_status_t** const  out_Status);
jcu_errorcode_t R_JCU_SetDecodeParam(const jcu_decode_param_t *const decode, const jcu_buffer_param_t *const buffer);
jcu_errorcode_t R_JCU_SetPauseForImageInfo(const bool_t is_pause);
jcu_errorcode_t R_JCU_GetImageInfo(jcu_image_info_t *const buffer);
jcu_errorcode_t R_JCU_SetErrorFilter(jcu_int_detail_errors_t filter);
jcu_errorcode_t R_JCU_SetQuantizationTable(const jcu_table_no_t tableNo, const uint8_t *const table);
jcu_errorcode_t R_JCU_SetHuffmanTable(const jcu_table_no_t tableNo, const jcu_huff_t type, const uint8_t *const table);
jcu_errorcode_t R_JCU_SetEncodeParam(const jcu_encode_param_t *const encode, const jcu_buffer_param_t *const buffer);
jcu_errorcode_t R_JCU_GetEncodedSize(size_t *const out_Size);
jcu_errorcode_t R_JCU_Set2ndCacheAttribute( r_ospl_axi_cache_attribute_t const  read_cache_attribute,
        r_ospl_axi_cache_attribute_t const  write_cache_attribute );

#ifndef R_OSPL_NDEBUG
void            R_JCU_PrintRegisters(void);
#endif

/* For interrupt callback */
errnum_t        R_JCU_OnInterrupting( const r_ospl_interrupt_t* const  InterruptSource );
errnum_t        R_JCU_OnInterrupted(void);

#endif /* #define JCU_API_H_ */
