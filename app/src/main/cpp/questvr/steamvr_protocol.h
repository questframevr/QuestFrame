//
//
//
#pragma once

#include <stdint.h>

#define QDVR_SOCKET_NAME "questdeck-steamvr"
#define QDVR_PROTOCOL_VERSION 1
#define QDVR_MAX_BUFFERS 3

enum {
    QDVR_MSG_HELLO = 1,
    QDVR_MSG_TRACKING = 2,
    QDVR_MSG_NEED_BUFFERS = 3,
    QDVR_MSG_BUFFERS = 4,
    QDVR_MSG_FRAME = 5,
    QDVR_MSG_HAPTIC = 6,
};

typedef struct {
    float px, py, pz;
    float qx, qy, qz, qw;
} qdvr_pose;

typedef struct {
    float left, right, up, down;
} qdvr_fov;

typedef struct {
    uint32_t type;  // QDVR_MSG_HELLO
    uint32_t version;
    uint32_t eyeWidth, eyeHeight;
    float refreshHz;
    qdvr_fov fov[2];
    qdvr_pose eyeToHead[2];
} qdvr_hello;

enum {
    QDVR_HAND_ACTIVE = 1 << 0,
    QDVR_HAND_POSE_VALID = 1 << 1,
};

enum {
    QDVR_BTN_A = 1 << 0,
    QDVR_BTN_B = 1 << 1,
    QDVR_BTN_SYSTEM = 1 << 2,
    QDVR_BTN_STICK = 1 << 3,
    QDVR_BTN_TRIGGER = 1 << 4,
    QDVR_BTN_GRIP = 1 << 5,
};

typedef struct {
    uint32_t flags;    // QDVR_HAND_*
    uint32_t buttons;  // QDVR_BTN_* held
    uint32_t touches;
    float trigger, grip, stickX, stickY;
    qdvr_pose gripPose, aimPose;
} qdvr_hand;

typedef struct {
    uint32_t type;  // QDVR_MSG_TRACKING
    uint32_t seq;
    int64_t displayTimeNs;
    uint32_t headValid;
    float frameIntervalSec;
    qdvr_pose head;
    qdvr_hand hands[2];  // left, right
} qdvr_tracking;

typedef struct {
    uint32_t type;  // QDVR_MSG_NEED_BUFFERS
    uint32_t eyeWidth, eyeHeight;
    uint32_t vkFormat;
} qdvr_need_buffers;

typedef struct {
    uint32_t type;
    uint32_t generation;
    uint32_t count;
    uint32_t width, height;
    uint32_t strideBytes;
    uint32_t drmFourcc;
    uint32_t pad;
    uint64_t modifier;  // DRM_FORMAT_MOD_LINEAR or DRM_FORMAT_MOD_QCOM_COMPRESSED
    uint64_t sizeBytes;
} qdvr_buffers;

typedef struct {
    uint32_t type;  // QDVR_MSG_FRAME
    uint32_t generation;
    uint32_t index;
    uint32_t trackingSeq;
    qdvr_pose renderHead;
    uint32_t srgb;
    uint32_t pad;
} qdvr_frame;

typedef struct {
    uint32_t type;  // QDVR_MSG_HAPTIC
    uint32_t hand;
    float durationSec, frequency, amplitude;
} qdvr_haptic;
