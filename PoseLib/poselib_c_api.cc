// Copyright (c) 2024
// C API wrapper implementation for PoseLib

#include "PoseLib/poselib_c_api.h"
#include "PoseLib/robust.h"
#include "PoseLib/misc/quaternion.h"

#include <vector>
#include <cstring>
#include <limits>

// ============================================================================
// Helper functions to create default options
// ============================================================================

extern "C" {

POSELIB_API void poselib_get_default_ransac_options(PoseLibRansacOptions* options) {
    if (!options) return;
    
    options->max_iterations = 100000;
    options->min_iterations = 1000;
    options->dyn_num_trials_mult = 3.0;
    options->success_prob = 0.9999;
    options->max_reproj_error = 12.0;
    options->max_epipolar_error = 1.0;
    options->seed = 0;
    options->progressive_sampling = 0;
    options->max_prosac_iterations = 100000;
    options->real_focal_check = 0;
    options->score_initial_model = 0;
    options->monodepth_estimate_shift = 0;
    options->monodepth_weight_sampson = 1.0f;
}

POSELIB_API void poselib_get_default_bundle_options(PoseLibBundleOptions* options) {
    if (!options) return;
    
    options->max_iterations = 100;
    options->loss_type = POSELIB_LOSS_CAUCHY;
    options->loss_scale = 1.0;
    options->gradient_tol = 1e-10;
    options->step_tol = 1e-8;
    options->initial_lambda = 1e-3;
    options->min_lambda = 1e-10;
    options->max_lambda = 1e10;
    options->verbose = 0;
}

// ============================================================================
// Internal conversion helpers
// ============================================================================

static poselib::RansacOptions convert_ransac_options(const PoseLibRansacOptions* opt) {
    poselib::RansacOptions result;
    
    if (!opt) return result;  // Return defaults
    
    result.max_iterations = opt->max_iterations;
    result.min_iterations = opt->min_iterations;
    result.dyn_num_trials_mult = opt->dyn_num_trials_mult;
    result.success_prob = opt->success_prob;
    result.max_reproj_error = opt->max_reproj_error;
    result.max_epipolar_error = opt->max_epipolar_error;
    result.seed = opt->seed;
    result.progressive_sampling = (opt->progressive_sampling != 0);
    result.max_prosac_iterations = opt->max_prosac_iterations;
    result.real_focal_check = (opt->real_focal_check != 0);
    result.score_initial_model = (opt->score_initial_model != 0);
    result.monodepth_estimate_shift = (opt->monodepth_estimate_shift != 0);
    result.monodepth_weight_sampson = opt->monodepth_weight_sampson;
    
    return result;
}

static poselib::BundleOptions convert_bundle_options(const PoseLibBundleOptions* opt) {
    poselib::BundleOptions result;
    
    if (!opt) return result;  // Return defaults
    
    result.max_iterations = opt->max_iterations;
    result.loss_type = static_cast<poselib::BundleOptions::LossType>(opt->loss_type);
    result.loss_scale = opt->loss_scale;
    result.gradient_tol = opt->gradient_tol;
    result.step_tol = opt->step_tol;
    result.initial_lambda = opt->initial_lambda;
    result.min_lambda = opt->min_lambda;
    result.max_lambda = opt->max_lambda;
    result.verbose = (opt->verbose != 0);
    
    return result;
}

static poselib::Camera convert_camera(const PoseLibCamera* cam) {
    poselib::Camera result;
    
    if (!cam) return result;
    
    result.model_id = cam->model_id;
    result.width = cam->width;
    result.height = cam->height;
    
    if (cam->params && cam->num_params > 0) {
        result.params.assign(cam->params, cam->params + cam->num_params);
    }
    
    return result;
}

static void convert_stats_out(const poselib::RansacStats& stats, PoseLibRansacStats* out) {
    if (!out) return;
    
    out->refinements = stats.refinements;
    out->iterations = stats.iterations;
    out->num_inliers = stats.num_inliers;
    out->inlier_ratio = stats.inlier_ratio;
    out->model_score = stats.model_score;
}

static void convert_pose_out(const poselib::CameraPose& pose, PoseLibCameraPose* out) {
    if (!out) return;
    
    out->q[0] = pose.q(0);
    out->q[1] = pose.q(1);
    out->q[2] = pose.q(2);
    out->q[3] = pose.q(3);
    
    out->t[0] = pose.t(0);
    out->t[1] = pose.t(1);
    out->t[2] = pose.t(2);
}

// ============================================================================
// Main estimation functions
// ============================================================================

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
) {
    // Validate required inputs
    if (!points2d || !points3d || num_points == 0 || !camera || !pose_out) {
        return -1;  // Invalid arguments
    }
    
    try {
        // Convert 2D points from interleaved array to vector of Eigen::Vector2d
        std::vector<poselib::Point2D> pts2d(num_points);
        for (size_t i = 0; i < num_points; ++i) {
            pts2d[i] = Eigen::Vector2d(points2d[i * 2], points2d[i * 2 + 1]);
        }
        
        // Convert 3D points from interleaved array to vector of Eigen::Vector3d
        std::vector<poselib::Point3D> pts3d(num_points);
        for (size_t i = 0; i < num_points; ++i) {
            pts3d[i] = Eigen::Vector3d(points3d[i * 3], points3d[i * 3 + 1], points3d[i * 3 + 2]);
        }
        
        // Convert options
        poselib::RansacOptions ransac_options = convert_ransac_options(ransac_opt);
        poselib::BundleOptions bundle_options = convert_bundle_options(bundle_opt);
        poselib::Camera cam = convert_camera(camera);
        
        // Prepare output variables
        poselib::CameraPose pose;
        std::vector<char> inliers;
        
        // Call the actual PoseLib function
        poselib::RansacStats stats = poselib::estimate_absolute_pose(
            pts2d, pts3d, cam, ransac_options, bundle_options, &pose, &inliers
        );
        
        // Convert outputs
        convert_pose_out(pose, pose_out);
        
        if (inliers_out && inliers.size() == num_points) {
            std::memcpy(inliers_out, inliers.data(), num_points);
        }
        
        if (stats_out) {
            convert_stats_out(stats, stats_out);
        }
        
        return 0;  // Success
        
    } catch (const std::exception& e) {
        return -2;  // Exception occurred
    } catch (...) {
        return -3;  // Unknown exception
    }
}

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
) {
    // Set up camera
    PoseLibCamera camera;
    camera.model_id = camera_model_id;
    camera.width = image_width;
    camera.height = image_height;
    camera.params = camera_params;
    camera.num_params = num_camera_params;
    
    // Set up RANSAC options with custom max_reproj_error
    PoseLibRansacOptions ransac_opt;
    poselib_get_default_ransac_options(&ransac_opt);
    ransac_opt.max_reproj_error = max_reproj_error;
    
    // Use default bundle options
    PoseLibBundleOptions bundle_opt;
    poselib_get_default_bundle_options(&bundle_opt);
    
    // For stats
    PoseLibRansacStats stats;
    
    int result = poselib_estimate_absolute_pose(
        points2d, points3d, num_points,
        &camera, &ransac_opt, &bundle_opt,
        pose_out, nullptr, &stats
    );
    
    if (num_inliers_out && result == 0) {
        *num_inliers_out = stats.num_inliers;
    }
    
    return result;
}

// ============================================================================
// Utility functions
// ============================================================================

POSELIB_API int poselib_get_camera_model_num_params(int model_id) {
    switch (model_id) {
        case POSELIB_CAMERA_SIMPLE_PINHOLE: return 3;  // f, cx, cy
        case POSELIB_CAMERA_PINHOLE: return 4;         // fx, fy, cx, cy
        case POSELIB_CAMERA_SIMPLE_RADIAL: return 4;   // f, cx, cy, k1
        case POSELIB_CAMERA_RADIAL: return 5;          // f, cx, cy, k1, k2
        case POSELIB_CAMERA_OPENCV: return 8;          // fx, fy, cx, cy, k1, k2, p1, p2
        case POSELIB_CAMERA_OPENCV_FISHEYE: return 8;  // fx, fy, cx, cy, k1, k2, k3, k4
        case POSELIB_CAMERA_FULL_OPENCV: return 12;    // fx, fy, cx, cy, k1, k2, p1, p2, k3, k4, k5, k6
        default: return -1;
    }
}

POSELIB_API const char* poselib_get_camera_model_name(int model_id) {
    switch (model_id) {
        case POSELIB_CAMERA_NULL: return "NULL";
        case POSELIB_CAMERA_SIMPLE_PINHOLE: return "SIMPLE_PINHOLE";
        case POSELIB_CAMERA_PINHOLE: return "PINHOLE";
        case POSELIB_CAMERA_SIMPLE_RADIAL: return "SIMPLE_RADIAL";
        case POSELIB_CAMERA_RADIAL: return "RADIAL";
        case POSELIB_CAMERA_OPENCV: return "OPENCV";
        case POSELIB_CAMERA_OPENCV_FISHEYE: return "OPENCV_FISHEYE";
        case POSELIB_CAMERA_FULL_OPENCV: return "FULL_OPENCV";
        default: return "UNKNOWN";
    }
}

POSELIB_API void poselib_rotmat_to_quat(const double* rotmat, double* quat_out) {
    if (!rotmat || !quat_out) return;
    
    // Convert row-major array to Eigen matrix
    Eigen::Matrix3d R;
    R << rotmat[0], rotmat[1], rotmat[2],
         rotmat[3], rotmat[4], rotmat[5],
         rotmat[6], rotmat[7], rotmat[8];
    
    Eigen::Vector4d q = poselib::rotmat_to_quat(R);
    
    quat_out[0] = q(0);
    quat_out[1] = q(1);
    quat_out[2] = q(2);
    quat_out[3] = q(3);
}

POSELIB_API void poselib_quat_to_rotmat(const double* quat, double* rotmat_out) {
    if (!quat || !rotmat_out) return;
    
    Eigen::Vector4d q(quat[0], quat[1], quat[2], quat[3]);
    Eigen::Matrix3d R = poselib::quat_to_rotmat(q);
    
    // Convert to row-major array
    rotmat_out[0] = R(0, 0); rotmat_out[1] = R(0, 1); rotmat_out[2] = R(0, 2);
    rotmat_out[3] = R(1, 0); rotmat_out[4] = R(1, 1); rotmat_out[5] = R(1, 2);
    rotmat_out[6] = R(2, 0); rotmat_out[7] = R(2, 1); rotmat_out[8] = R(2, 2);
}

} // extern "C"
