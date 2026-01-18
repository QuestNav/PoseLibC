// Copyright (c) 2024
// C API wrapper for PoseLib - enables use from C# via P/Invoke

#ifndef POSELIB_C_API_H_
#define POSELIB_C_API_H_

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Platform-specific export macros
#if defined(_WIN32) || defined(_WIN64)
    #ifdef POSELIB_BUILDING_DLL
        #define POSELIB_API __declspec(dllexport)
    #else
        #define POSELIB_API __declspec(dllimport)
    #endif
#else
    #define POSELIB_API __attribute__((visibility("default")))
#endif

// ============================================================================
// Enums
// ============================================================================

typedef enum {
    POSELIB_LOSS_TRIVIAL = 0,
    POSELIB_LOSS_TRUNCATED = 1,
    POSELIB_LOSS_HUBER = 2,
    POSELIB_LOSS_CAUCHY = 3,
    POSELIB_LOSS_TRUNCATED_CAUCHY = 4,
    POSELIB_LOSS_TRUNCATED_LE_ZACH = 5
} PoseLibLossType;

typedef enum {
    POSELIB_CAMERA_NULL = -1,
    POSELIB_CAMERA_SIMPLE_PINHOLE = 0,
    POSELIB_CAMERA_PINHOLE = 1,
    POSELIB_CAMERA_SIMPLE_RADIAL = 2,
    POSELIB_CAMERA_RADIAL = 3,
    POSELIB_CAMERA_OPENCV = 4,
    POSELIB_CAMERA_OPENCV_FISHEYE = 5,
    POSELIB_CAMERA_FULL_OPENCV = 6
} PoseLibCameraModelId;

// ============================================================================
// Structs
// ============================================================================

// RANSAC options for pose estimation
typedef struct {
    size_t max_iterations;
    size_t min_iterations;
    double dyn_num_trials_mult;
    double success_prob;
    double max_reproj_error;
    double max_epipolar_error;
    unsigned long seed;
    int progressive_sampling;      // bool as int for C compatibility
    size_t max_prosac_iterations;
    int real_focal_check;          // bool as int
    int score_initial_model;       // bool as int
    int monodepth_estimate_shift;  // bool as int
    float monodepth_weight_sampson;
} PoseLibRansacOptions;

// RANSAC statistics (returned from estimation)
typedef struct {
    size_t refinements;
    size_t iterations;
    size_t num_inliers;
    double inlier_ratio;
    double model_score;
} PoseLibRansacStats;

// Bundle adjustment options
typedef struct {
    size_t max_iterations;
    PoseLibLossType loss_type;
    double loss_scale;
    double gradient_tol;
    double step_tol;
    double initial_lambda;
    double min_lambda;
    double max_lambda;
    int verbose;  // bool as int
} PoseLibBundleOptions;

// Camera intrinsics
typedef struct {
    int model_id;
    int width;
    int height;
    const double* params;
    size_t num_params;
} PoseLibCamera;

// Camera pose (rotation quaternion + translation)
typedef struct {
    double q[4];  // Quaternion: [QW, QX, QY, QZ]
    double t[3];  // Translation: [X, Y, Z]
} PoseLibCameraPose;

// ============================================================================
// Helper functions to create default options
// ============================================================================

// Get default RANSAC options
POSELIB_API void poselib_get_default_ransac_options(PoseLibRansacOptions* options);

// Get default bundle adjustment options
POSELIB_API void poselib_get_default_bundle_options(PoseLibBundleOptions* options);

// ============================================================================
// Main estimation functions
// ============================================================================

/**
 * Estimate absolute camera pose using LO-RANSAC followed by non-linear refinement.
 *
 * @param points2d      Pointer to 2D points, stored as [x0, y0, x1, y1, ...] (interleaved)
 * @param points3d      Pointer to 3D points, stored as [x0, y0, z0, x1, y1, z1, ...] (interleaved)
 * @param num_points    Number of point correspondences
 * @param camera        Camera intrinsics
 * @param ransac_opt    RANSAC options
 * @param bundle_opt    Bundle adjustment options
 * @param pose_out      Output: estimated camera pose
 * @param inliers_out   Output: inlier mask (array of num_points chars, 0 or 1). Can be NULL.
 * @param stats_out     Output: RANSAC statistics. Can be NULL.
 *
 * @return 0 on success, non-zero on error
 */
POSELIB_API int poselib_estimate_absolute_pose(
    const double* points2d,
    const double* points3d,
    size_t num_points,
    const PoseLibCamera* camera,
    const PoseLibRansacOptions* ransac_opt,
    const PoseLibBundleOptions* bundle_opt,
    PoseLibCameraPose* pose_out,
    char* inliers_out,
    PoseLibRansacStats* stats_out
);

/**
 * Simplified version with default options.
 * Uses default RANSAC and bundle options.
 */
POSELIB_API int poselib_estimate_absolute_pose_simple(
    const double* points2d,
    const double* points3d,
    size_t num_points,
    int camera_model_id,
    int image_width,
    int image_height,
    const double* camera_params,
    size_t num_camera_params,
    double max_reproj_error,
    PoseLibCameraPose* pose_out,
    size_t* num_inliers_out
);

// ============================================================================
// Utility functions
// ============================================================================

// Get the number of parameters required for a camera model
POSELIB_API int poselib_get_camera_model_num_params(int model_id);

// Get camera model name from ID
POSELIB_API const char* poselib_get_camera_model_name(int model_id);

// Convert rotation matrix (row-major, 9 doubles) to quaternion
POSELIB_API void poselib_rotmat_to_quat(const double* rotmat, double* quat_out);

// Convert quaternion to rotation matrix (row-major, 9 doubles)
POSELIB_API void poselib_quat_to_rotmat(const double* quat, double* rotmat_out);

#ifdef __cplusplus
}
#endif

#endif // POSELIB_C_API_H_
