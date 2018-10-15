/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef FRAME_H
#define FRAME_H

#include<vector>

#include "MapPoint.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"
#include "ORBVocabulary.h"
#include "KeyFrame.h"
#include "ORBextractor.h"

#include <opencv2/opencv.hpp>

namespace ORB_SLAM2
{
#define FRAME_GRID_ROWS 48
#define FRAME_GRID_COLS 64

class MapPoint;
class KeyFrame;


/**
* Represents a frame, an image, with the detected keypoints.
*
* Frame represents a camera pose, with its image, detected 2D points, its descriptors, its 3D mapping, pose and others.
*
* The frame's position on the map is obtained with Frame::GetCameraCenter. The orientation is obtained with Frame::GetRotationInverse.
*
* Tracking uses 3 frames:
* - Frame::mCurrentFrame
* - Frame::mInitialFrame
* - Frame::mLastFrame
*
* Each frame has its own K calibration matrix
* It is usually the same matrix (with the same values) for every frame and keyframes.
*
* The constructor clones Mat K, making a copy in Frame::mK.
* Then, it analyzes but does not save the image. It is lost when the constructor ends its execution.
* Part of the analysis consists in detecting keypoints, extracting its descriptors and classifying them.
*
* Keypoints, descriptors, BoW and tracked 3D points are registered in parallel vectors and explained in Frame::N.
*
* The coordinate system and the meaning of the position matrices is explained in Frame::mTcw.
*
* This class does not determine the frame's pose. Its pose is registered by Tracking and Optimizer.
*
* The matrix's pose is explained in Frame::mTcw.
*
* \sa SetPose
* \sa mTcw
*/
class Frame
{
public:

        /**
         * The empty arguments constructor creates an uninitialized Frame
         * The only initialized data is nNextId=0.
         * Not used.
         */
    Frame();

    // Copy constructor. Clones a frame
    Frame(const Frame &frame);

    // Constructor for stereo cameras.
    Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timeStamp, ORBextractor* extractorLeft, ORBextractor* extractorRight, ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth);

    // Constructor for RGB-D cameras.
    Frame(const cv::Mat &imGray, const cv::Mat &imDepth, const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth);

     /**
     * Constructor that craetes a Frame and initializes it with the arguments.
     * @param timeStamp Timestamp for the registry. ORB-SLAM does not use it.
     * @param extractor Extractor algorithm used to obtain descriptors.  ORB-SLAM uses BRIEF extractor by ORB specifically.
     *
     * Two camera modes are distinguished: normal when the distortion coefficients are provided, or fisheye if noArray() is provided.
     *
     * It is only Invoked from Tracling::GrabImageMonocular.
     */
    // Constructor for Monocular cameras.
    Frame(const cv::Mat &imGray, const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth);

    /**
     * Proceeds with the ORB descriptors extraction.
     *
     * @param flag false for monocular, or left camera.  true for right camera.  It is always invoked with false.
     * @param im Image from which the descriptors are extracted.
     *
     * The descriptors are preserved in Frame::mDescriptors.
     *
     * Only invoked from constructor.
     */
    // Extract ORB on the image. 0 for left image and 1 for right image.
    void ExtractORB(int flag, const cv::Mat &im);

    /**
     * Computes BoW for every descriptor in the frame.
     * They are saved in the mBowVec property of type BowVector and in mFeatVec of type FeatureVector.
     *
     * If mBowVec is non-empty, the method returns without doing anything, avoiding recomputation.
     *
     * BoW is computed for a frame when it is built in keyframe, and when trying to relocate.
     *
     * DBoW2 is documented in http://webdiis.unizar.es/~dorian/doc/dbow2/annotated.html
     *
     *
     */
    // Compute Bag of Words representation.
    void ComputeBoW();

    /**
     * Register the pose .
     *
     * Used by multiple methods to establish or correct the frame's pose.
     *
     * After establishing the new pose its different representations are recalculated with UpdatePoseMatrices, such as the translation vector or the rotation matrix.
     *
     * @param Tcw New pose, rototranslation matrix in homogeneous coordinates of 4x4.
     *
     * Registers the new pose in Frame::mTcw, which belongs to the coordinate system's point of origin's pose of the camera.
     *
     * This method is invoked by Tracking with approximate positions to initialize and track down,
     * and by Optimizer::PoseOptimization, which is the only one that registers the optimized pose.
     */
    // Set the camera pose.
    void SetPose(cv::Mat Tcw);

    /**
     * Calculates the mRcw mTcw and mOw position matrices based on the mTcw pose.
     * This matrices are a way of exposing the pose, they are not used in the ORB-SLAM operation.
     * UpdatePoseMatrices() extracts the information from mTcw, which is the matrix that combines the complete pose.
     *
     */
    // Computes rotation, translation and camera center matrices from the camera pose.
    void UpdatePoseMatrices();

    /** Returns the camera position's vector, the center of the camera.*/
    // Returns the camera center.
    inline cv::Mat GetCameraCenter(){
        return mOw.clone();
    }

    /**
     * Returns the orientation on the map.
     *
     * It is the inverse of the mRwc rotation.
     *
     * @returns mRwc.t()
     *
     * It does not use GetRotation to return the rotation without inverting mRcw.
     */
    // Returns inverse of rotation
    inline cv::Mat GetRotationInverse(){
        return mRwc.clone();
    }

    /**
     * Indicates if a given 3D point belong to the frame's visual subspace (frustum).
     * The visual subspace is a quadrilateral based pyramid, which vertices are those in the frame but undistorted.
     * viewingCosLimit is a way of limiting the frustum's scope.
     */
    // Check if a MapPoint is in the frustum of the camera
    // and fill variables of the MapPoint to be used by the tracking
    bool isInFrustum(MapPoint* pMP, float viewingCosLimit);

    /**
     * Calculates the coordinates of the grid's cell, to which a keypoint belongs.
     * Informs the coordinates in the posX and posY arguments passed by reference.
     * Returns true if the point belong to the grid, otherwise false.
     * @param kp Undistorted keypoint.
     * @param posX X coordinate of the keypoint's cell.
     * @param posY Y coordinate of the keypoint's cell.
     *
     */
    // Compute the cell of a keypoint (return false if outside the grid)
    bool PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY);

    /**
     * Selects the points inside a squared window of center x,y and r radius (2r side).
     *
     * Traverses every level of the frame, filtering the points by coordinates.
     *
     * Used to reduce the matching candidates.
     *
     * @param x Center of the area X coordinate
     * @param y Center of the area Y coordinate
     * @param r Radius of the squared area
     * @param minLevel Minimum level of the pyramid to search for the keypoints. Negative if there is no minimum.
     * @param maxLevel Maximum level of the pyramid to search for the keypoints. Negative if there is no maximum.
     *
     * Only invoked by multiple methods in ORBmatcher.
     */
    vector<size_t> GetFeaturesInArea(const float &x, const float  &y, const float  &r, const int minLevel=-1, const int maxLevel=-1) const;

    // Search a match for each keypoint in the left image to a keypoint in the right image.
    // If there is a match, depth is computed and the right coordinate associated to the left keypoint is stored.
    void ComputeStereoMatches();

    // Associate a "right" coordinate to a keypoint if there is valid depth in the depthmap.
    void ComputeStereoFromRGBD(const cv::Mat &imDepth);

    // Backprojects a keypoint (if stereo/depth info available) into 3D world coordinates.
    cv::Mat UnprojectStereo(const int &i);

public:
    /** BOW Vocabulary to classify descriptors.*/
    // Vocabulary used for relocalization.

    /** Algorithm used to extract descriptors. The framework allows the developer to test different extractors; ORB-SLAM only uses ORBextractor.*/
    ORBVocabulary* mpORBvocabulary;

    /** Extractor used to extract descriptors*/
    // Feature extractor. The right is used only in the stereo case.
    ORBextractor* mpORBextractorLeft, *mpORBextractorRight;

    /**
     * Image capture's Timestamp.
     * Only for registry and research purposes, it is not used by the algorithm.
     */
    // Frame timestamp.
    double mTimeStamp;

    /**
     * Intrinsic K Matrix, from the camera.
     * Distortion coefficients mDistCoef, intrinsic parameters fx, fy, cx, cy.
     */
    // Calibration matrix and OpenCV distortion parameters.
    cv::Mat mK;

    /** Intrinsic parametersfx, fy, cx, cy.*/
    /** Intrinsic parameter.*/
    static float fx;
    /** Intrinsic parameter.*/
    static float fy;
    /** Intrinsic parameter.*/
    static float cx;
    /** Intrinsic parameter.*/
    static float cy;
    /** Inverse of the intrinsic parameter.*/
    static float invfx;
    /** Intrinsic parameter.*/
    static float invfy;

    /** Distortion coefficients of the camera mDistCoef.*/
    cv::Mat mDistCoef;

    /** Not used in monocular.*/
    // Stereo baseline multiplied by fx.
    float mbf;

    /** Not used in monocular.*/
    // Stereo baseline in meters.
    float mb;

    /** Not used in monocular.*/
    // Threshold close/far points. Close points are inserted from 1 view.
    // Far points are inserted as in the monocular case from 2 views.
    float mThDepth;

    /**
	 * Amount of keypoints.
	 *
	 * Size of the paired vectors:
	 * - mvKeys
	 * - mvKeysUn
	 * - mDescriptors
	 * - mBowVec
	 * - mFeatVec
	 * - mvpMapPoints
	 * - mvbOutlier
	 *
	 *
	 * All these are passed to the keyframe.
	 */
    // Number of KeyPoints.
    int N;

    // Vector of keypoints (original for visualization) and undistorted (actually used by the system).
    // In the stereo case, mvKeysUn is redundant as images must be rectified.
    // In the RGB-D case, RGB images can be distorted.
    /**
     * Vector of keypoints obtained by the detector, as returned by opencv.
     *
     * Its coordinates are expressed in pixels, in the image's reference system.
     */
    std::vector<cv::KeyPoint> mvKeys, mvKeysRight;

    /**
     * Vector of undistorted points, mvKeys corrected according to the distortion coefficients.
     *
     * This vector is paired with mvKeys, both size N.
     *
     * Its coordinates are expressed in pixels, in the image's reference system.
     * The points are obtained by cv::undistortPoints, reapplying the camera's K matrix.
     */
    std::vector<cv::KeyPoint> mvKeysUn;

    // Corresponding stereo coordinate and depth for each keypoint.
    // "Monocular" keypoints have a negative value.
    std::vector<float> mvuRight;

    /** -1 for monocular.  It is passed in the Frame's copy constructor.*/
    std::vector<float> mvDepth;

    // Bag of Words Vector structures.
    /**
     * BoW vector correspondent to the keypoints.
     *
     * BowVector is a Word Id(unsigned int) -> Word value (double) map, which represents a weight.
     */
    DBoW2::BowVector mBowVec;

    /**
     * "Feature" Vector correspondent to the keypoints.
     */
    DBoW2::FeatureVector mFeatVec;

    /** ORB descriptors in Mat format, as returned by opencv.  mDescritorRight is not used, passed in the copy constructor.*/
    // ORB descriptor, each row associated to a keypoint.
    cv::Mat mDescriptors, mDescriptorsRight;

    /** Map's 3D points Vector, associated to the keypoints.
	 * This vector has the same length as mvKeys and mvKeysUn.
	 * Unassociated position are NULL.
	 */
    // MapPoints associated to keypoints, NULL pointer if no association.
    std::vector<MapPoint*> mvpMapPoints;

    /** Flag that indicates if mvpMapPoints contains associated outliers.*/
    // Flag to identify outlier associations.
    std::vector<bool> mvbOutlier;

    /** mfGridElementWidthInv is the inverse of the cell's width in pixels.
	 * The points of the undistorted image are divided in cells using a FRAME_GRID_COLS by FRAME_GRID_ROWS grid,
	 * to reduce the matching complexity.
	 */
    // Keypoints are assigned to cells in a grid to reduce matching complexity when projecting MapPoints.
    static float mfGridElementWidthInv;

    /** mfGridElementHeightInv is the inverse of the cell's height in pixels .
     * The points of the undistorted image are divided in cells using a FRAME_GRID_COLS by FRAME_GRID_ROWS grid,
     * to reduce the matching complexity.
     */
    static float mfGridElementHeightInv;
    std::vector<std::size_t> mGrid[FRAME_GRID_COLS][FRAME_GRID_ROWS];

    /**
     * Camera pose.
     * 4x4 rototranslation matrix in homogeneous coordinates.
     * It is cv:Mat(4,4,CV_32F), and its elements are type float.
     *
     * The camera position(translation vector) is obtained from the last column of this matrix.
     *
     * The unit of measure is established in the initialization according to the scene's average depth.
     * This means that the unit depends on the triangulated 3D points in the initialization.
     *
     * The 3D coordinate system of a camera is related to its projection 2D
     * keeping the X and Y axis in parallel, and establishing Z forward.
     * In this way X points to the right and Y points downwards (contrary to the standard coordinate system).
     * This coordinate system is known as _optical:  http://www.ros.org/reps/rep-0103.html#axis-orientation
     * The homogeneous 3D vectors have the traditional disposition:
     *
     * V = [vx, vy, vz, 1]t
     *
     * The poses are 4x4 matrices as this one, its subindeces indicate reference and subject.  Tcw is T in respect to the camera, the world.
     * The poses are combined in this way:
     *
     * Tca = Tba * Tcb
     *
     * Its value is updated through SetPose, which also extracts the rotation matrix and the translation vector with UpdatePoseMatrices.
     * These extracted data is calculated for presentation, but they are not used by the algorithm, that are only used by mTcw.
     *
     * Frame does not calculate mTcw.  This matrix is calculated and passed to the frame using SetPose, in:
     * - Tracking, copying a previous frame, initializing the cartesian pose, or estimating through the movement model.
     * - Optimizer::PoseOptimization, invoked from multiple Tracking methods.  This is where the real pose calculation is made.
     *
     */
    // Camera pose.
    cv::Mat mTcw;

    /** Incremental Id with the value for the next frame.
	 * Its a class variable that keeps record of the id number.
	 * A frame's id is assigned through
	 *     mnId=nNextId++;
	 */
    // Current and Next Frame id.
    static long unsigned int nNextId;

    /** Incremental Id that identifies this frame.*/
    long unsigned int mnId;

    /** Reference KeyFrame.*/
    // Reference Keyframe.
    KeyFrame* mpReferenceKF;

    /** Amount of levels in the pyramid.*/
    // Scale pyramid info.
    int mnScaleLevels;

    /** Scale factor between levels of the pyramid.*/
    float mfScaleFactor;

    /** Logarithmic scale factor.*/
    float mfLogScaleFactor;

    /** Scale factors for each pyramid level vector.*/
    vector<float> mvScaleFactors;

    /** Inverse of mvScaleFactors.*/
    vector<float> mvInvScaleFactors;

    /** Square of mvScaleFactos.*/
    vector<float> mvLevelSigma2;

    /** Inverse of mvLevelSigma2.*/
    vector<float> mvInvLevelSigma2;

    /** Undistorted image vertices: mnMinX, mnMinY, mnMaxX, mnMaxY.*/
    // Undistorted Image Bounds (computed once).
    static float mnMinX;

    /** Undistorted image vertices: mnMinX, mnMinY, mnMaxX, mnMaxY.*/
    static float mnMaxX;

    /** Undistorted image vertices: mnMinX, mnMinY, mnMaxX, mnMaxY.*/
    static float mnMinY;

    /** Undistorted image vertices: mnMinX, mnMinY, mnMaxX, mnMaxY.*/
    static float mnMaxY;

    /**
	 * This variable is set to one with a constructor argument only when the first frame is created.
	 * true for class variable calculations, that are not changed afterwards.
	 */
    static bool mbInitialComputations;


private:

    /**
     * Calculates the keypoints of mvKeysUn.
     *
     * Undistorts detected points, from mvKeys, and saves them in mvKeysUn in the same order.
     * If there is no distortion, UndistortKeyPoints quickly returns unifying mvKeysUn = mvKeys in the same vector.
     */
    // Undistort keypoints given OpenCV distortion parameters.
    // Only for the RGB-D case. Stereo must be already rectified!
    // (called in the constructor).
    void UndistortKeyPoints();

    /**
     * Calculates the vertices of the undistorted frame.
     * Define mnMinX, mnMaxX, mnMinY, mnMaxY.
     * If there is no distortion, the result is trivial with origin at  (0,0).
     *
     * @param imLeft Image, with the only purpose of measuring its size using  .rows and .cols .
     *
     * Only invoked from the constructor.
     */
    // Computes image bounds for the undistorted image (called in the constructor).
    void ComputeImageBounds(const cv::Mat &imLeft);

    /**
     * Assigns the keypoints to the corresponding cells in the grid.
     * The image is divided in a grid to detect points in a more homogeneous way.
     * After undistort the detected keypoints' coordinates,
     * this method creates a vector of keypoints for each cell in the grid, and populates it.
     */
    // Assign keypoints to the grid for speed up feature matching (called in the constructor).
    void AssignFeaturesToGrid();

    // Rotation, translation and camera center

    /**
     * R matrix of rotation of the world respect to the camera.
     * It is updated with UpdatePoseMatrices().
     */
    cv::Mat mRcw;

    /**
     * Vector t of translation from the world's origin in the camera's reference system.
     * It is updated with UpdatePoseMatrices().
     */
    cv::Mat mtcw;

    /**
     * Inverse of the rotation matrix, de la cámara respecto del mundo.
     * It is updated with UpdatePoseMatrices().
     */
    cv::Mat mRwc;

    /**
     * Center of camera vector, camera's position respect to the world.
     *
     * It is private, informed withFrame::GetCameraCenter.
     *
     * 3x1 Matrix (vertical vector).
     * Inverse translation vector.  mtcw is the translation vector of the world's origin in the camera's reference system.
     * It is updated with UpdatePoseMatrices().
     */
    cv::Mat mOw; //==mtwc
};

}// namespace ORB_SLAM

#endif // FRAME_H
