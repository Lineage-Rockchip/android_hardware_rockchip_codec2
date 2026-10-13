/*
 * Copyright (C) 2020 Rockchip Electronics Co. LTD
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "rk_mpi.h"
#include "C2RKNaluParser.h"
#include "C2RKLogger.h"

namespace android {

C2_LOGGER_ENABLE("C2RKNaluParser");

#define H264_NALU_TYPE_SPS              7
#define H264_PROFILE_IDC_HIGH10       110
#define H265_MAX_VPS_COUNT             16
#define H265_MAX_SUB_LAYERS             7
#define H265_PROFILE_IDC_MAIN_10        2
#define H265_NALU_TYPE_VPS             32
#define H265_NALU_TYPE_SPS             33

/* find h2645 start code */
int32_t findStartCode(uint8_t *buf, int32_t size) {
    if (size < 4)
        return 0;

    // start code: 0x000001 or 0x00000001
    if (buf[0] == 0x00 && buf[1] == 0x00 && buf[2] == 0x01) {
        return 3;
    } else if (buf[0] == 0x00 && buf[1] == 0x00 && buf[2] == 0x00 && buf[3] == 0x01) {
        return 4;
    } else {
        // TODO(@TEAM) loop find start code
        return 0;
    }
}

bool C2RKNaluParser::searchAVCNaluInfo(
        uint8_t *buf, int32_t size, int32_t detectFiled, int32_t *outValue) {
    BitReadContext  gbCtx;
    BitReadContext *gb = &gbCtx;
    int32_t startCodeLen = 0;
    int32_t val = 0, i = 0;
    int32_t profileIdc = 0, chromaFormatIdc = 1;

    c2_set_bitread_ctx(gb, buf, size);
    c2_set_pre_detection(gb);
    if (!c2_update_curbyte(gb)) {
        Log.E("failed to update curbyte, skipping.");
        goto error;
    }

    /*
     * ExtraData carry h264 sps_info in two ways.
     * 1. start with 0x000001 or 0x00000001
     * 2. AVC extraData configuration
     */

    startCodeLen = findStartCode(buf, size);
    if (startCodeLen > 0) {
        SKIP_BITS(gb, startCodeLen * 8);
    } else {
        // AVC extraData configuration
        SKIP_BITS(gb, 32);
        SKIP_BITS(gb, 16);

        SKIP_BITS(gb, 16);  // sequenceParameterSetLength
    }

    /* parse h264 sps info */
    READ_ONEBIT(gb, &val);  // forbidden_bit

    SKIP_BITS(gb, 2);  // nal_ref_idc
    READ_BITS(gb, 5, &val);  // nalu_type
    // stop traversal if not SPS nalu type
    if (val != H264_NALU_TYPE_SPS) {
        goto error;
    }

    static uint8_t zzScan[16] =
    { 0, 1, 4, 8, 5, 2, 3, 6, 9, 12, 13, 10, 7, 11, 14, 15 };

    static uint8_t zzScan8[64] = {
        0, 1, 8, 16, 9, 2, 3, 10, 17, 24, 32, 25, 18, 11, 4, 5,
        12, 19, 26, 33, 40, 48, 41, 34, 27, 20, 13, 6, 7, 14, 21, 28,
        35, 42, 49, 56, 57, 50, 43, 36, 29, 22, 15, 23, 30, 37, 44, 51,
        58, 59, 52, 45, 38, 31, 39, 46, 53, 60, 61, 54, 47, 55, 62, 63
    };

    READ_BITS(gb, 8, &profileIdc);  // profile_idc
    if (detectFiled == C2_DETECT_FIELD_DEPTH) {
        if (profileIdc == H264_PROFILE_IDC_HIGH10) {
            *outValue = 10;
        } else {
            *outValue = 8;
        }
        Log.D("get AVC stream bitDepth %d", (*outValue));
        return true;
    }

    SKIP_BITS(gb, 16);
    READ_UE(gb, &val);

    if (profileIdc == 100 || profileIdc == 110 ||
        profileIdc == 122 || profileIdc == 244 ||
        profileIdc == 44  || profileIdc == 83 ||
        profileIdc == 86  || profileIdc == 118 ||
        profileIdc == 128 || profileIdc == 138) {
        READ_UE(gb, &chromaFormatIdc);  // chroma_format_idc
        if (chromaFormatIdc > 2) goto __BR_ERR;

        READ_UE(gb, &val);  // bit_depth_luma_minus8
        if (val >= 7) goto __BR_ERR;

        READ_UE(gb, &val);  // bit_depth_chroma_minus8
        if (val >= 7) goto __BR_ERR;

        SKIP_BITS(gb, 1);
        READ_ONEBIT(gb, &val);  // seq_scaling_matrix_present_flag
        if (val) {
            int32_t seqScalingListPresentFlag[12];
            int32_t scalingList[64];

            // scaling_list4x4.
            for (i = 0; i < 6; i++) {
                READ_ONEBIT(gb, &seqScalingListPresentFlag[i]);
                if (seqScalingListPresentFlag[i]) {
                    int32_t scanj = 0, deltaScale = 0;
                    int32_t lastScale = 8, nextScale = 8;
                    int j = 0;
                    for (j = 0; j < 16; j++) {
                        scanj = zzScan[j];
                        if (nextScale != 0) {
                            READ_SE(gb, &deltaScale);
                            nextScale = (lastScale + deltaScale + 256) & 0xff;
                        }
                        scalingList[scanj] = (nextScale == 0) ? lastScale : nextScale;
                        lastScale = scalingList[scanj];
                    }
                }
            }

            // scaling_list8x8.
            for (i = 0; i < ((chromaFormatIdc != 3) ? 2 : 6); i++) {
                READ_ONEBIT(gb, &seqScalingListPresentFlag[6 + i]);
                if (seqScalingListPresentFlag[6 + i]) {
                    int32_t scanj = 0, deltaScale = 0;
                    int32_t lastScale = 8, nextScale = 8;
                    int j = 0;
                    for (j = 0; j < 64; j++) {
                        scanj = zzScan8[j];
                        if (nextScale != 0) {
                            READ_SE(gb, &deltaScale);
                            nextScale = (lastScale + deltaScale + 256) & 0xff;
                        }
                        scalingList[scanj] = (nextScale == 0) ? lastScale : nextScale;
                        lastScale = scalingList[scanj];
                    }
                }
            }
        }
    }

    READ_UE(gb, &val);  // log2_max_frame_num_minus4
    if (val >= 13) goto __BR_ERR;

    READ_UE(gb, &val);  // pic_order_cnt_type
    if (val == 0) {
        READ_UE(gb, &val);  // log2_max_pic_order_cnt_lsb_minus4
        if (val >= 13) goto __BR_ERR;
    } else if (val == 1) {
        READ_ONEBIT(gb, &val);
        READ_SE(gb, &val);
        READ_SE(gb, &val);
        READ_UE(gb, &val);  // num_ref_frames_in_pic_order_cnt_cycle
        for (i = 0; i < val; ++i) {
            READ_SE(gb, &val);
        }
    } else if (val >= 3) {
        goto __BR_ERR;
    }

    READ_UE(gb, &val);
    if (detectFiled == C2_DETECT_FIELD_MAX_REF_COUNT) {
        Log.D("get AVC stream maxRefCount %d", val);
        *outValue = val;
        return true;
    }

    if (detectFiled == C2_DETECT_FIELD_RESOLUTION) {
        int32_t widthMbs = 0, heightMapUnits = 0, frameMbsOnly = 0, cropping = 0;
        int32_t cropLeft = 0, cropRight = 0, cropTop = 0, cropBottom = 0;
        int32_t cropUnitX = 1, cropUnitY = 1;

        SKIP_BITS(gb, 1);  // gaps_in_frame_num_value_allowed_flag
        READ_UE(gb, &widthMbs);  // pic_width_in_mbs_minus1
        READ_UE(gb, &heightMapUnits);  // pic_height_in_map_units_minus1
        READ_ONEBIT(gb, &frameMbsOnly);  // frame_mbs_only_flag
        if (!frameMbsOnly) {
            SKIP_BITS(gb, 1);  // mb_adaptive_frame_field_flag
        }
        SKIP_BITS(gb, 1);  // direct_8x8_inference_flag
        READ_ONEBIT(gb, &cropping);  // frame_cropping_flag
        if (cropping) {
            READ_UE(gb, &cropLeft);
            READ_UE(gb, &cropRight);
            READ_UE(gb, &cropTop);
            READ_UE(gb, &cropBottom);
        }

        if (chromaFormatIdc == 1 || chromaFormatIdc == 2) {
            cropUnitX = 2;
        }
        cropUnitY = ((chromaFormatIdc == 1) ? 2 : 1) * (2 - frameMbsOnly);

        int32_t width  = (widthMbs + 1) * 16 - cropUnitX * (cropLeft + cropRight);
        int32_t height = (2 - frameMbsOnly) * (heightMapUnits + 1) * 16
                         - cropUnitY * (cropTop + cropBottom);
        if (width <= 0 || height <= 0 || width > 0xffff || height > 0xffff) {
            goto __BR_ERR;
        }

        Log.D("get AVC stream resolution %dx%d", width, height);
        *outValue = (width << 16) | height;
        return true;
    }

__BR_ERR:
error:
    return false;
}

bool C2RKNaluParser::searchHEVCNalSPS(
        BitReadContext *gb, int32_t detectFiled, int32_t *outValue) {
    int32_t val = 0;

    READ_BITS(gb, 4, &val); // vps-id
    if (val > H265_MAX_VPS_COUNT) {
        Log.E("VPS id out of range: %d", val);
        goto error;
    }

    READ_BITS(gb, 3, &val);
    val += 1;

    if (val > H265_MAX_SUB_LAYERS) {
        Log.E("sps_max_sub_layers out of range: %d", val);
        goto error;
    }

    SKIP_BITS(gb, 1); // temporal_id_nesting_flag

    SKIP_BITS(gb, 3); // profile_space & tier_flag
    READ_BITS(gb, 5, &val); // profile_idc

    if (detectFiled == C2_DETECT_FIELD_DEPTH) {
        if (val == H265_PROFILE_IDC_MAIN_10) {
            *outValue = 10;
        } else {
            *outValue = 8;
        }
        Log.D("get HEVC stream bitDepth %d", (*outValue));
        return true;
    }

__BR_ERR:
error:
    return false;
}

bool C2RKNaluParser::searchHEVCNalVPS(
        BitReadContext *gb, int32_t detectFiled, int32_t *outValue) {
    int32_t val = 0;
    int32_t vpsMaxSubLayers = -1;
    uint8_t subLayerProfilePresentFlag[7];
    uint8_t subLayerLevelPresentFlag[7];
    uint8_t vpsSubLayerOrderingInfoPresentFlag = 0;
    uint32_t vpsMaxDecPicBuffering[7];
    int32_t i = 0;

    READ_BITS(gb, 4, &val); // vps-id
    if (val >= H265_MAX_VPS_COUNT) {
        Log.E("VPS id out of range: %d", val);
        goto error;
    }

    READ_BITS(gb, 2, &val);
    if (val != 3) {  // vps_reserved_three_2bits
        Log.E("vps_reserved_three_2bits is not three");
        goto error;
    }

    SKIP_BITS(gb, 6);  // vps_max_layers

    READ_BITS(gb, 3, &vpsMaxSubLayers);
    vpsMaxSubLayers = vpsMaxSubLayers + 1;

    SKIP_BITS(gb, 1);  // vps_temporal_id_nesting_flag
    READ_BITS(gb, 16, &val);
    if (val != 0xffff) {  // vps_reserved_ffff_16bits
        Log.E("vps_reserved_ffff_16bits is not 0xffff");
        return 0;
    }

    if (vpsMaxSubLayers > 7) {
        Log.E("vps_max_sub_layers out of range: %d", vpsMaxSubLayers);
        return 0;
    }

    SKIP_BITS(gb, 88);  // profile_tier_level
    SKIP_BITS(gb, 8);  // general_ptl.level_idc

    for (i = 0; i < vpsMaxSubLayers - 1; i++) {
        READ_ONEBIT(gb, &subLayerProfilePresentFlag[i]);
        READ_ONEBIT(gb, &subLayerLevelPresentFlag[i]);
    }
    if (vpsMaxSubLayers - 1 > 0) {
        for (i = vpsMaxSubLayers - 1; i < 8; i++)
            SKIP_BITS(gb, 2);  // reserved_zero_2bits[i]
    }
    for (i = 0; i < vpsMaxSubLayers - 1; i++) {
        if (subLayerProfilePresentFlag[i]) {
            SKIP_BITS(gb, 88);  // profile_tier_level
        }
        if (subLayerLevelPresentFlag[i])
            SKIP_BITS(gb, 8);  // sub_layer_ptl[i].level_idc
    }

    READ_ONEBIT(gb, &vpsSubLayerOrderingInfoPresentFlag);

    i = vpsSubLayerOrderingInfoPresentFlag ? 0 : vpsMaxSubLayers - 1;
    for (; i < vpsMaxSubLayers; i++) {
        READ_UE(gb, &vpsMaxDecPicBuffering[i]);
        vpsMaxDecPicBuffering[i] = vpsMaxDecPicBuffering[i] + 1;

        if (detectFiled == C2_DETECT_FIELD_MAX_REF_COUNT) {
            *outValue += vpsMaxDecPicBuffering[i];
        }
        READ_UE(gb, &val);  // vps_num_reorder_pics
        READ_UE(gb, &val);  // vps_max_latency_increase

        if (vpsMaxDecPicBuffering[i] > 17) {
            Log.E("vpsMaxDecPicBuffering_minus1 out of range: %d",
                   vpsMaxDecPicBuffering[i] - 1);
            goto error;
        }
    }

    if (detectFiled == C2_DETECT_FIELD_MAX_REF_COUNT) {
        Log.D("get HEVC stream maxRefCount %d", (*outValue));
    }

    return true;

__BR_ERR:
error:
    *outValue = 0;
    return false;
}

bool C2RKNaluParser::searchHEVCNalUnit(
        uint8_t *buf, int32_t size, int32_t detectFiled, int32_t *outValue) {
    BitReadContext gb_ctx;
    BitReadContext *gb = &gb_ctx;
    int32_t nalUnitType = 0;
    int32_t nuhLayerId = 0;
    int32_t temporalId = 0;
    int32_t detectNaluType = H265_NALU_TYPE_SPS;

    if (detectFiled == C2_DETECT_FIELD_MAX_REF_COUNT) {
        detectNaluType = H265_NALU_TYPE_VPS;
    }

    c2_set_bitread_ctx(gb, buf, size);
    c2_set_pre_detection(gb);
    if (!c2_update_curbyte(gb)) {
        Log.E("failed to update curbyte, skipping.");
        return false;
    }

    SKIP_BITS(gb, 1); /* this bit should be zero */
    READ_BITS(gb, 6, &nalUnitType);
    READ_BITS(gb, 6, &nuhLayerId);
    READ_BITS(gb, 3, &temporalId);

    temporalId = temporalId -1;

    Log.D("nal_unit_type: %d, nuh_layer_id: %d temporal_id: %d",
           nalUnitType, nuhLayerId, temporalId);

    if (temporalId < 0) {
        Log.E("Invalid NAL unit %d, skipping.", nalUnitType);
        goto error;
    }

    if (nalUnitType != detectNaluType) {
        goto error;
    }

    switch (nalUnitType) {
        case H265_NALU_TYPE_SPS: {
            if (!searchHEVCNalSPS(gb, detectFiled, outValue)) goto error;
        } break;
        case H265_NALU_TYPE_VPS: {
            if (!searchHEVCNalVPS(gb, detectFiled, outValue)) goto error;
        } break;
        default: {
            Log.D("not support nalunit type %d", nalUnitType);
            goto error;
        } break;
    }

    return true;

__BR_ERR:
error:
    return false;
}

bool C2RKNaluParser::searchHEVCNaluInfo(
        uint8_t *buf, int32_t size, int32_t detectFiled, int32_t *outValue) {
    if (buf[0] || buf[1] || buf[2] > 1) {
        int32_t i = 0, j = 0;
        int32_t nalLenSize = 0;
        uint32_t numOfArrays = 0, numOfNals = 0;

        /* It seems the extradata is encoded as hvcC format.
           Temporarily, we support configurationVersion==0 until 14496-15 3rd
           is finalized. When finalized, configurationVersion will be 1 and we
           can recognize hvcC by checking if h265dctx->extradata[0]==1 or not. */
        if (size < 7) {
            goto error;
        }

        Log.D("extradata is encoded as hvcC format");

        nalLenSize = 1 + (buf[14 + 7] & 3);
        buf += 22;
        size -= 22;
        numOfArrays = static_cast<char>(buf[0]);
        buf += 1;
        size -= 1;
        for (i = 0; i < numOfArrays; i++) {
            buf += 1;
            size -= 1;
            // Num of nals
            numOfNals = buf[0] << 8 | buf[1];
            buf += 2;
            size -= 2;

            for (j = 0; j < numOfNals; j++) {
                uint32_t length = 0;
                if (size < 2) {
                    goto error;
                }

                length = buf[0] << 8 | buf[1];

                buf += 2;
                size -= 2;
                if (size < length) {
                    goto error;
                }
                if (searchHEVCNalUnit(buf, length, detectFiled, outValue)) {
                    return true;
                }
                buf += length;
                size -= length;
            }
        }
    } else {
        int32_t i = 0;
        int32_t detectNaluType = H265_NALU_TYPE_SPS;

        if (detectFiled == C2_DETECT_FIELD_MAX_REF_COUNT) {
            detectNaluType = H265_NALU_TYPE_VPS;
        }

        for (i = 0; i < size - 4; i++) {
            // find start code
            if (buf[i] == 0x00 && buf[i + 1] == 0x00 &&
                buf[i + 2] == 0x01 && ((buf[i + 3] & 0x7f) >> 1) == detectNaluType) {
                Log.D("find h265 start code");
                i += 3;
                if (searchHEVCNalUnit(buf + i, size - i, detectFiled, outValue)) {
                    return true;
                }

            }
        }
    }

error:
    return false;
}

int32_t C2RKNaluParser::detectBitDepth(uint8_t *buf, int32_t size, int32_t coding) {
    int32_t bitDepth = 8;

    switch (coding) {
        case MPP_VIDEO_CodingAVC: {
            if (!searchAVCNaluInfo(buf, size, C2_DETECT_FIELD_DEPTH, &bitDepth)) {
                bitDepth = 8;
                Log.D("failed to find bitDepth, set default 8bit");
            }
        } break;
        case MPP_VIDEO_CodingHEVC: {
            if (!searchHEVCNaluInfo(buf, size, C2_DETECT_FIELD_DEPTH, &bitDepth)) {
                bitDepth = 8;
                Log.D("failed to find bitDepth, set default 8bit");
            }
        } break;
        default: {
            bitDepth = 8;
            Log.D("not support coding %d, set default 8bit", coding);
        } break;
    }
    return bitDepth;
}

int32_t C2RKNaluParser::detectMaxRefCount(uint8_t *buf, int32_t size, int32_t coding) {
    int32_t maxRefCount = 0;

    switch (coding) {
        case MPP_VIDEO_CodingAVC: {
            if (!searchAVCNaluInfo(
                    buf, size, C2_DETECT_FIELD_MAX_REF_COUNT, &maxRefCount)) {
                maxRefCount = 0;
                Log.D("failed to find maxRefCount");
            }
        } break;
        case MPP_VIDEO_CodingHEVC: {
            if (!searchHEVCNaluInfo(
                    buf, size, C2_DETECT_FIELD_MAX_REF_COUNT, &maxRefCount)) {
                maxRefCount = 0;
                Log.D("failed to find maxRefCount");
            }
        } break;
        default: {
            maxRefCount = 0;
            Log.D("not support coding %d", coding);
        } break;
    }
    return maxRefCount;
}

namespace {

/* plain MSB-first reader for AV1 and VP9 headers, which have no emulation prevention */
struct C2RKPlainBits {
    const uint8_t *data;
    int64_t        bits;
    int64_t        pos;
    bool           overrun;

    C2RKPlainBits(const uint8_t *buf, int32_t size)
        : data(buf), bits((int64_t)size * 8), pos(0), overrun(false) {}

    uint32_t read(int32_t n) {
        uint32_t v = 0;
        while (n-- > 0) {
            if (pos >= bits) {
                overrun = true;
                return 0;
            }
            v = (v << 1) | ((data[pos >> 3] >> (7 - (pos & 7))) & 1);
            pos++;
        }
        return v;
    }

    uint32_t readUvlc() {
        int32_t leadingZeros = 0;
        while (!overrun && !read(1)) {
            if (++leadingZeros >= 32) {
                overrun = true;
                return 0;
            }
        }
        if (leadingZeros == 0) {
            return 0;
        }
        return read(leadingZeros) + (1u << leadingZeros) - 1;
    }
};

bool readLeb128(const uint8_t *buf, int32_t size, int32_t *consumed, uint64_t *value) {
    uint64_t v = 0;
    for (int32_t i = 0; i < 8 && i < size; i++) {
        v |= (uint64_t)(buf[i] & 0x7f) << (i * 7);
        if (!(buf[i] & 0x80)) {
            *consumed = i + 1;
            *value = v;
            return true;
        }
    }
    return false;
}

bool parseAV1SequenceHeader(
        const uint8_t *buf, int32_t size, int32_t *width, int32_t *height) {
    C2RKPlainBits br(buf, size);

    br.read(3);  // seq_profile
    br.read(1);  // still_picture
    uint32_t reducedStillPictureHeader = br.read(1);

    if (reducedStillPictureHeader) {
        br.read(5);  // seq_level_idx[0]
    } else {
        uint32_t decoderModelInfoPresent = 0;
        uint32_t bufferDelayLength = 0;

        if (br.read(1)) {  // timing_info_present_flag
            br.read(32);  // num_units_in_display_tick
            br.read(32);  // time_scale
            if (br.read(1)) {  // equal_picture_interval
                br.readUvlc();  // num_ticks_per_picture_minus_1
            }
            decoderModelInfoPresent = br.read(1);
            if (decoderModelInfoPresent) {
                bufferDelayLength = br.read(5) + 1;
                br.read(32);  // num_units_in_decoding_tick
                br.read(5);  // buffer_removal_time_length_minus_1
                br.read(5);  // frame_presentation_time_length_minus_1
            }
        }

        uint32_t initialDisplayDelayPresent = br.read(1);
        uint32_t operatingPoints = br.read(5) + 1;
        for (uint32_t i = 0; i < operatingPoints && !br.overrun; i++) {
            br.read(12);  // operating_point_idc
            if (br.read(5) > 7) {  // seq_level_idx
                br.read(1);  // seq_tier
            }
            if (decoderModelInfoPresent && br.read(1)) {
                br.read(bufferDelayLength);  // decoder_buffer_delay
                br.read(bufferDelayLength);  // encoder_buffer_delay
                br.read(1);  // low_delay_mode_flag
            }
            if (initialDisplayDelayPresent && br.read(1)) {
                br.read(4);  // initial_display_delay_minus_1
            }
        }
    }

    int32_t widthBits  = br.read(4) + 1;
    int32_t heightBits = br.read(4) + 1;
    int32_t w = (int32_t)br.read(widthBits) + 1;
    int32_t h = (int32_t)br.read(heightBits) + 1;

    if (br.overrun || w <= 1 || h <= 1) {
        return false;
    }

    *width = w;
    *height = h;
    return true;
}

}  // namespace

bool C2RKNaluParser::searchAVCResolution(
        uint8_t *buf, int32_t size, int32_t *width, int32_t *height) {
    for (int32_t i = 0; i + 4 < size; i++) {
        if (buf[i] != 0 || buf[i + 1] != 0 || buf[i + 2] != 1) {
            continue;
        }
        int32_t naluType = buf[i + 3] & 0x1f;
        if (naluType == H264_NALU_TYPE_SPS) {
            int32_t value = 0;
            if (searchAVCNaluInfo(buf + i, size - i, C2_DETECT_FIELD_RESOLUTION, &value)) {
                *width = value >> 16;
                *height = value & 0xffff;
                return true;
            }
        } else if (naluType >= 1 && naluType <= 5) {
            // an SPS comes before the slices of its access unit
            break;
        }
        i += 2;
    }

    /* AVC extraData configuration (avcC) */
    if (size > 8 && buf[0] == 1) {
        int32_t value = 0;
        if (searchAVCNaluInfo(buf, size, C2_DETECT_FIELD_RESOLUTION, &value)) {
            *width = value >> 16;
            *height = value & 0xffff;
            return true;
        }
    }

    return false;
}

bool C2RKNaluParser::searchAV1Resolution(
        uint8_t *buf, int32_t size, int32_t *width, int32_t *height) {
    int32_t pos = 0;

    /* AV1CodecConfigurationRecord: marker 1, version 1, then configOBUs */
    if (size > 4 && buf[0] == 0x81) {
        pos = 4;
    }

    while (pos < size) {
        uint8_t header = buf[pos];
        int32_t obuType = (header >> 3) & 0xf;
        int32_t hasExtension = (header >> 2) & 1;
        int32_t hasSize = (header >> 1) & 1;
        int32_t payloadPos = pos + 1 + hasExtension;
        uint64_t payloadSize = 0;

        if ((header & 0x80) || payloadPos > size) {
            return false;
        }

        if (hasSize) {
            int32_t consumed = 0;
            if (!readLeb128(buf + payloadPos, size - payloadPos, &consumed, &payloadSize)) {
                return false;
            }
            payloadPos += consumed;
        } else {
            payloadSize = size - payloadPos;
        }

        if (payloadSize > (uint64_t)(size - payloadPos)) {
            return false;
        }

        if (obuType == 1 /* OBU_SEQUENCE_HEADER */) {
            return parseAV1SequenceHeader(buf + payloadPos, (int32_t)payloadSize, width, height);
        }

        pos = payloadPos + (int32_t)payloadSize;
    }

    return false;
}

bool C2RKNaluParser::searchVP9Resolution(
        uint8_t *buf, int32_t size, int32_t *width, int32_t *height) {
    C2RKPlainBits br(buf, size);

    if (br.read(2) != 2) {  // frame_marker
        return false;
    }

    uint32_t profile = br.read(1);
    profile |= br.read(1) << 1;
    if (profile == 3) {
        br.read(1);  // reserved_zero
    }

    if (br.read(1)) {  // show_existing_frame
        return false;
    }

    if (br.read(1) != 0) {  // frame_type, only key frames carry the size
        return false;
    }

    br.read(1);  // show_frame
    br.read(1);  // error_resilient_mode

    if (br.read(8) != 0x49 || br.read(8) != 0x83 || br.read(8) != 0x42) {
        return false;
    }

    if (profile >= 2) {
        br.read(1);  // ten_or_twelve_bit
    }
    if (br.read(3) != 7 /* CS_RGB */) {
        br.read(1);  // color_range
        if (profile == 1 || profile == 3) {
            br.read(3);  // subsampling_x, subsampling_y, reserved_zero
        }
    } else if (profile == 1 || profile == 3) {
        br.read(1);  // reserved_zero
    }

    int32_t w = (int32_t)br.read(16) + 1;
    int32_t h = (int32_t)br.read(16) + 1;
    if (br.overrun) {
        return false;
    }

    *width = w;
    *height = h;
    return true;
}

bool C2RKNaluParser::detectResolution(
        uint8_t *buf, int32_t size, int32_t coding, int32_t *width, int32_t *height) {
    bool found = false;

    if (buf == nullptr || size <= 0) {
        return false;
    }

    switch (coding) {
    case MPP_VIDEO_CodingAVC:
        found = searchAVCResolution(buf, size, width, height);
        break;
    case MPP_VIDEO_CodingAV1:
        found = searchAV1Resolution(buf, size, width, height);
        break;
    case MPP_VIDEO_CodingVP9:
        found = searchVP9Resolution(buf, size, width, height);
        break;
    default:
        break;
    }

    if (found) {
        Log.D("detected coded resolution %dx%d", *width, *height);
    }

    return found;
}

} // namespace android