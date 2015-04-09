/*
* Copyright 2014 Google Inc. All Rights Reserved.
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



#include "tango_data.h"
static float prev_depth_timestamp = 0.0f;

// Get status string based on the pose status code.
static const char* getStatusStringFromStatusCode(TangoPoseStatusType status) {
  const char* status_string = nullptr;
  switch (status) {
    case TANGO_POSE_INITIALIZING:
      status_string = "initializing";
      break;
    case TANGO_POSE_VALID:
      status_string = "valid";
      break;
    case TANGO_POSE_INVALID:
      status_string = "invalid";
      break;
    case TANGO_POSE_UNKNOWN:
      status_string = "unknown";
      break;
    default:
      break;
  }
  return status_string;
}

static void onFrameAvailable(void* context, TangoCameraId id, const TangoImageBuffer* buffer) {
  // Copy TangoImageBuffer
  if(!buffer){
    return;
  }
  if(!buffer->data){
    return;
  }
  LOGI("onFrameAvailable called");
  // Allocate the image buffer that we're going to push on the queue
  TangoImageBuffer * next_enqueue = new TangoImageBuffer();
  next_enqueue->width = buffer->width;
  next_enqueue->height = buffer->height;
  next_enqueue->stride = buffer->stride;
  next_enqueue->timestamp = buffer->timestamp;
  next_enqueue->frame_number = buffer->frame_number;
  next_enqueue->format = buffer->format;
  next_enqueue->data = new uint8_t[buffer->width * buffer->height * 4];
  memcpy(next_enqueue->data, buffer->data,
	  buffer->width * buffer->height * 4*sizeof(*(buffer->data)));
  if(id == TANGO_CAMERA_FISHEYE){
    // Lock Mutex
    pthread_mutex_lock(&TangoData::GetInstance().fisheye_mutex);
    delete next_enqueue;
    // Enqueue
    // TangoData::GetInstance().fisheye_queue.push(next_enqueue);
    pthread_cond_signal(&TangoData::GetInstance().fisheye_cv);
    // Signal condition variable
    pthread_mutex_unlock(&TangoData::GetInstance().fisheye_mutex);
  } else if (TANGO_CAMERA_COLOR){
    // Lock Mutex
    pthread_mutex_lock(&TangoData::GetInstance().frame_mutex);
    delete next_enqueue;
    // Enqueue
    // TangoData::GetInstance().frame_queue.push(next_enqueue);
    pthread_cond_signal(&TangoData::GetInstance().frame_cv);
    // Signal condition variable
    pthread_mutex_unlock(&TangoData::GetInstance().frame_mutex);
  }


}

/// Callback function when new XYZij data available, caller is responsible
/// for allocating the memory, and the memory will be released after the
/// callback function is over.
/// XYZij data updates in 5Hz.
static void onXYZijAvailable(void*, const TangoXYZij* XYZ_ij) {
  TangoXYZij * next_enqueue = new TangoXYZij();
  next_enqueue->ij_rows = XYZ_ij->ij_rows;
  next_enqueue->ij_cols = XYZ_ij->ij_cols;
  next_enqueue->xyz_count = XYZ_ij->xyz_count;
  next_enqueue->timestamp = XYZ_ij->timestamp;
  next_enqueue->xyz = new float[XYZ_ij->xyz_count][3];
  next_enqueue->ij = new uint32_t[XYZ_ij->ij_cols * XYZ_ij->ij_rows];

  pthread_mutex_lock(&TangoData::GetInstance().xyzij_mutex);
  memcpy(next_enqueue->xyz, XYZ_ij->xyz,XYZ_ij->xyz_count * 3 * sizeof(float));
	  
  memcpy(next_enqueue->ij, XYZ_ij->ij,XYZ_ij->ij_cols * XYZ_ij->ij_rows*sizeof(uint32_t));
	  
  // Enqueue (TODO)
  TangoData::GetInstance().xyzij_queue.push(next_enqueue);
  pthread_cond_signal(&TangoData::GetInstance().xyzij_cv);

  // Copying out the depth buffer.
  // Note: the XYZ_ij object will be out of scope after this callback is
  // excuted.
  if (XYZ_ij->xyz_count <= TangoData::GetInstance().max_vertex_count) {
    if (TangoData::GetInstance().depth_buffer != nullptr &&
        XYZ_ij->xyz != nullptr) {
      memcpy(TangoData::GetInstance().depth_buffer, XYZ_ij->xyz,
             XYZ_ij->xyz_count * 3 * sizeof(float));
    }
  }
  TangoData::GetInstance().depth_buffer_size = XYZ_ij->xyz_count;

  // Calculate the depth delta frame time, and store current and
  // previous frame timestamp. prev_depth_timestamp used for querying
  // closest pose data. (See in UpdateXYZijData())
  TangoData::GetInstance().depth_frame_delta_time =
      (XYZ_ij->timestamp - prev_depth_timestamp) * kSecondToMillisecond;
  prev_depth_timestamp = XYZ_ij->timestamp;

  // Set xyz_ij dirty flag.
  TangoData::GetInstance().is_xyzij_dirty = true;

  pthread_mutex_unlock(&TangoData::GetInstance().xyzij_mutex);
}

// Tango event callback.
static void onTangoEvent(void*, const TangoEvent* event) {
  // Update the status string for debug display.
  std::stringstream string_stream;
  string_stream << event->event_key << ": " << event->event_value;
  TangoData::GetInstance().event_string = string_stream.str();
  if(event->type == TANGO_EVENT_COLOR_CAMERA)
	  LOGI("Tango Color Camera Event fired");

}

// This callback function is called when new pose update is available.
static void onPoseAvailable(void*, const TangoPoseData* pose) {
  pthread_mutex_lock(&TangoData::GetInstance().pose_mutex);
  if(pose != nullptr){
    if (pose->frame.base == TANGO_COORDINATE_FRAME_START_OF_SERVICE) {
      TangoData::GetInstance().cur_pose_data = *pose;
      TangoData::GetInstance().is_pose_dirty = true;
    } else if (pose->frame.base == TANGO_COORDINATE_FRAME_PREVIOUS_DEVICE_POSE) {
	    TangoPoseData *next_enqueue = new TangoPoseData;
	    //next_enqueue->orientation = new double[4];
	    next_enqueue->orientation[0] = pose->orientation[0];
	    next_enqueue->orientation[1] = pose->orientation[1];
	    next_enqueue->orientation[2] = pose->orientation[2];
	    next_enqueue->orientation[3] = pose->orientation[3];
	    //next_enqueue->translation = new translation[3];
	    next_enqueue->translation[0] = pose->translation[0];
	    next_enqueue->translation[1] = pose->translation[1];
	    next_enqueue->translation[2] = pose->translation[2];
	    next_enqueue->status_code = pose->status_code;
	    next_enqueue->frame = pose->frame;
	    next_enqueue->timestamp = pose->timestamp;
	    TangoData::GetInstance().pose_queue.push(next_enqueue);
	    pthread_cond_signal(&TangoData::GetInstance().pose_cv);
    }
  }

  pthread_mutex_unlock(&TangoData::GetInstance().pose_mutex);
}

void * record_pose(void*){
  TangoPoseData * current_pose;
  pthread_mutex_lock(&TangoData::GetInstance().pose_mutex);
  pthread_cond_wait(&TangoData::GetInstance().pose_cv, &TangoData::GetInstance().pose_mutex);
  string file_path = *TangoData::GetInstance().pose_external_path + string("/pose.csv");
  std::ofstream fp;
  fp.open(file_path.c_str());
  LOGI("Record Pose Started. File: %s",file_path.c_str());
  while(!TangoData::GetInstance().pose_terminate){
    while(!TangoData::GetInstance().pose_queue.size()){
        pthread_cond_wait(&TangoData::GetInstance().pose_cv, &TangoData::GetInstance().pose_mutex);
    }
    current_pose = TangoData::GetInstance().pose_queue.front();
    if(current_pose == nullptr)
      LOGI("current_pose is null, queue size is %d",TangoData::GetInstance().pose_queue.size());
    TangoData::GetInstance().pose_queue.pop();
    fp << current_pose->timestamp << ",";
    fp << current_pose->orientation[0] << "," << current_pose->orientation[1] << ",";
    fp << current_pose->orientation[2] << "," << current_pose->orientation[3] << ",";
    fp << current_pose->translation[0] << "," << current_pose->translation[1] << "," << current_pose->translation[2] << "\n";
  }
  fp.close();
  pthread_mutex_unlock(&TangoData::GetInstance().pose_mutex);
}

void * record_frame(void*){
  pthread_mutex_lock(&TangoData::GetInstance().frame_mutex);
  pthread_cond_wait(&TangoData::GetInstance().frame_cv, &TangoData::GetInstance().frame_mutex);
  // TODO: Wait for first signal to open file
  LOGI("Record Frame Started");
  while(!TangoData::GetInstance().frame_terminate){
    while(!TangoData::GetInstance().frame_queue.size())
        pthread_cond_wait(&TangoData::GetInstance().frame_cv, &TangoData::GetInstance().frame_mutex);
  }
  pthread_mutex_unlock(&TangoData::GetInstance().frame_mutex);
}

void * record_fisheye(void*){
  pthread_mutex_lock(&TangoData::GetInstance().fisheye_mutex);
  pthread_cond_wait(&TangoData::GetInstance().fisheye_cv, &TangoData::GetInstance().fisheye_mutex);
  // TODO: Wait for first signal to open file
  LOGI("Record fisheye Started");
  while(!TangoData::GetInstance().fisheye_terminate){
    while(!TangoData::GetInstance().fisheye_queue.size())
        pthread_cond_wait(&TangoData::GetInstance().fisheye_cv, &TangoData::GetInstance().fisheye_mutex);
  }
  pthread_mutex_unlock(&TangoData::GetInstance().fisheye_mutex);
}

std::string to_str(double timestamp){
  std::ostringstream stm ;
  stm << timestamp ;
  return stm.str();
}

void write_xyzb_file(std::string path, TangoXYZij * xyzij){
      std::ofstream fp(path + string("/xyzij_") + to_str(xyzij->timestamp) + string(".xyz"),std::ios::out|std::ios::binary);
      for(int i = 0; i < xyzij->xyz_count; i++){
        fp.write((char*)xyzij->xyz[i], 3*sizeof(*xyzij->xyz[i]));
      }
      fp.close();
}

void write_xyz_file(std::string path, TangoXYZij * xyzij){
  std::ofstream fp(path + string("/xyzij_") + to_str(xyzij->timestamp) + string(".xyz"),std::ios::out);
  for(int i = 0; i < xyzij->xyz_count; i++){
    fp << xyzij->xyz[i][0] << " " << xyzij->xyz[i][1] << " " << xyzij->xyz[i][2] << "\n";
  }
  fp.close();
}

void write_ply_file(std::string path, TangoXYZij * xyzij){
  pcl::PointCloud<pcl::PointXYZ> xyz;
  for(int i = 0; i < xyzij->xyz_count; i++){
    xyz.push_back(pcl::PointXYZ(xyzij->xyz[i][0],xyzij->xyz[i][1], xyzij->xyz[i][2]));
  }
  pcl::io::savePLYFileBinary(path + string("/xyzij_") + to_str(xyzij->timestamp) + string(".ply"),xyz);
}

//TODO: How do we use the scan name? Can we change the camera focus?
void write_calibration(string path, string scan_name,
    TangoCameraIntrinsics calibration){
  string file_name =  path + string("/")+ scan_name + string(".calib");
  std::ofstream fp;
  fp.open(file_name.c_str());
  fp << "Camera id: " << calibration.camera_id << '\n';
  fp << "Calibration Type: " << calibration.calibration_type << '\n';
  fp << "Width (px): " << calibration.width << '\n';
  fp << "Height (px): " << calibration.height << '\n';
  fp << "Focal Length X: " << calibration.fx << '\n';
  fp << "Focal Length Y: " << calibration.fy << '\n';
  fp << "Prinicpal X (px): " << calibration.cx << '\n';
  fp << "Principal Y (px): " << calibration.cy << '\n';
  fp << "Distortion[0]: " << calibration.distortion[0] << '\n';
  fp << "Distortion[1]: " << calibration.distortion[1] << '\n';
  fp << "Distortion[2]: " << calibration.distortion[2] << '\n';
  fp << "Distortion[3]: " << calibration.distortion[3] << '\n';
  fp << "Distortion[4]: " << calibration.distortion[4] << '\n';
  fp.close();
}

void * record_xyzij(void*){
  TangoXYZij * current_xyzij;
  pthread_mutex_lock(&TangoData::GetInstance().xyzij_mutex);
  pthread_cond_wait(&TangoData::GetInstance().xyzij_cv, &TangoData::GetInstance().xyzij_mutex);
  // Write Camera Calibration Parameters
  string file_path = *TangoData::GetInstance().pose_external_path + string("/calibration.csv");
  std::ofstream fp;
  fp.open(file_path.c_str());
  while(!TangoData::GetInstance().xyzij_terminate){
    while(!TangoData::GetInstance().xyzij_queue.size()){
        pthread_cond_wait(&TangoData::GetInstance().xyzij_cv, &TangoData::GetInstance().xyzij_mutex);
    }
    current_xyzij = TangoData::GetInstance().xyzij_queue.front();
    if(current_xyzij == nullptr)
      LOGI("current_xyzij is null, queue size is %d",TangoData::GetInstance().xyzij_queue.size());
    TangoData::GetInstance().xyzij_queue.pop();
    write_ply_file(*TangoData::GetInstance().xyzij_external_path,current_xyzij);
    //current_xyzij->xyz
    delete[] current_xyzij->xyz;
    delete[] current_xyzij->ij;
    delete[] current_xyzij;
  }
  pthread_mutex_unlock(&TangoData::GetInstance().xyzij_mutex);
}
// Initialize Tango Service.
TangoErrorType TangoData::Initialize(JNIEnv* env, jobject activity) {
  // The initialize function perform API and Tango Service version check,
  // if there is a mis-match between API and Tango Service version, the
  // function will return TANGO_INVALID.
  return TangoService_initialize(env, activity);
}

TangoData::TangoData() : config_(nullptr) {
  is_xyzij_dirty = false;
  is_pose_dirty = false;

  frame_external_path = nullptr;
  xyzij_external_path = nullptr;
  pose_external_path = nullptr;

  pthread_create(&pose_thread, nullptr, record_pose, nullptr);
  pthread_create(&xyzij_thread, nullptr, record_xyzij, nullptr);
  //pthread_create(&frame_thread, nullptr, record_frame, nullptr);
  pthread_create(&fisheye_thread, nullptr, record_fisheye, nullptr);
  d_2_ss_mat_motion = glm::mat4(1.0f);
  d_2_ss_mat_depth = glm::mat4(1.0f);
  d_2_imu_mat = glm::mat4(1.0f);
  c_2_imu_mat = glm::mat4(1.0f);
}

// Set up Tango Configuration handle, and connecting all callbacks.
bool TangoData::SetConfig() {
  // Get the default TangoConfig.
  config_ = TangoService_getConfig(TANGO_CONFIG_DEFAULT);
  if (config_ == NULL) {
    LOGE("TangoService_getConfig(): Failed");
    return false;
  }

  // Enable depth.
  if (TangoConfig_setBool(config_, "config_enable_depth", true) !=
      TANGO_SUCCESS) {
    LOGE("config_enable_depth Failed");
    return false;
  }
  // Enable Color.

  if (TangoConfig_setBool(config_, "config_enable_color_camera", true) !=
      TANGO_SUCCESS) {
    LOGE("config_enable_color_camera Failed");
    return false;
  }


  // Get library version string from service.
  if (TangoConfig_getString(
          config_, "tango_service_library_version",
          const_cast<char*>(
              TangoData::GetInstance().lib_version_string.c_str()),
          kVersionStringLength) != TANGO_SUCCESS) {
    LOGE("Get tango_service_library_version Failed");
    return false;
  }

  // Get max point cloud elements. The value is used for allocating
  // the depth buffer.
  int temp = 0;
  if (TangoConfig_getInt32(config_, "max_point_cloud_elements", &temp) !=
      TANGO_SUCCESS) {
    LOGE("Get max_point_cloud_elements Failed");
    return false;
  }
  max_vertex_count = static_cast<uint32_t>(temp);

  // Forward allocate the maximum size of depth buffer.
  // max_vertex_count is the vertices count, max_vertex_count*3 is
  // the actual float buffer size.
  depth_buffer = new float[3 * max_vertex_count];

  return true;
}

bool TangoData::setExternalStorageDirectory(string id, string path) {
  LOGI("Setting External Storage Directory for ID %s to: %s", id.c_str(), path.c_str());
  string * ext_path = new string(path);
  if(id == string("TANGO_CAMERA_COLOR")){
    frame_external_path = ext_path;
  } else if(id == string("TANGO_CAMERA_FISHEYE")){
    fisheye_external_path = ext_path;
  } else if(id == string("TANGO_CAMERA_DEPTH")){
    xyzij_external_path = ext_path;
  } else if(id == string("TANGO_POSE")){
    pose_external_path = ext_path;
  } else {
    LOGI("Unknown ID: %s", id.c_str());
  }
  return true;
}

bool TangoData::ConnectCallbacks() {
  // Attach the onXYZijAvailable callback.
  int count = -1;
  cudaGetDeviceCount(&count);
  LOGI("Found %d cuda Devices",count); // TODO: Check driver?
  if (TangoService_connectOnXYZijAvailable(onXYZijAvailable) != TANGO_SUCCESS) {
    LOGI("TangoService_connectOnXYZijAvailable(): Failed");
    return false;
  }

  // Attach the frameAvailable callback.
  if ((TangoService_connectOnFrameAvailable(TANGO_CAMERA_COLOR,
    (void*) frame_external_path, onFrameAvailable)) != TANGO_SUCCESS) {
    LOGI("TangoService_connectOnFrameAvailable(): Failed");
    return false;
  }
    // Attach the frameAvailable callback.
    if ((TangoService_connectOnFrameAvailable(TANGO_CAMERA_FISHEYE,
      (void*) frame_external_path, onFrameAvailable)) != TANGO_SUCCESS) {
      LOGI("TangoService_connectOnFrameAvailable(): Failed");
      return false;
    }
  // Set the reference frame pair after connect to service.
  // Currently the API will set this set below as default.
  TangoCoordinateFramePair pairs[2];
  pairs[0].base = TANGO_COORDINATE_FRAME_START_OF_SERVICE;
  pairs[0].target = TANGO_COORDINATE_FRAME_DEVICE;

  // To align point clouds we're more worried about the frame-to-frame pose
  pairs[1].base = TANGO_COORDINATE_FRAME_PREVIOUS_DEVICE_POSE;
  pairs[1].target = TANGO_COORDINATE_FRAME_DEVICE;
  // Attach onPoseAvailable callback.
  if (TangoService_connectOnPoseAvailable(2, pairs, onPoseAvailable) !=
      TANGO_SUCCESS) {
    LOGI("TangoService_connectOnPoseAvailable(): Failed");
    return false;
  }

  // Set the event callback listener.
  if (TangoService_connectOnTangoEvent(onTangoEvent) != TANGO_SUCCESS) {
    LOGI("TangoService_connectOnTangoEvent(): Failed");
    return false;
  }
  return true;
}

// Connect to Tango Service
TangoErrorType TangoData::Connect() {
  return TangoService_connect(nullptr, config_);
}

// Disconnect from Tango Service.
void TangoData::Disconnect() {
  // Disconnect application from Tango Service.
  TangoService_disconnect();
}

// Update pose data. This function will be called only when the pose
// data is changed (dirty). This function is being called through the
// GL rendering thread (See tango_pointcloud.cpp, RenderFrame()).
//
// This will off load some computation inside the onPoseAvailable()
// callback function. Heavy computation inside callback will block the whole
// Tango Service callback thread, so migrating heavy computation to other
// thread is suggested.
void TangoData::UpdatePoseData() {
  pthread_mutex_lock(&pose_mutex);

  glm::vec3 tango_position =
      glm::vec3(cur_pose_data.translation[0], cur_pose_data.translation[1],
                cur_pose_data.translation[2]);
  glm::quat tango_rotation =
      glm::quat(cur_pose_data.orientation[3], cur_pose_data.orientation[0],
                cur_pose_data.orientation[1], cur_pose_data.orientation[2]);

  // Calculate status code count for debug display.
  if (prev_pose_data.status_code != cur_pose_data.status_code) {
    pose_status_count = 0;
  }
  ++pose_status_count;

  // Calculate frame delta time for debug display.
  // Note: this is the pose callback frame delta time.
  pose_frame_delta_time = (cur_pose_data.timestamp - prev_pose_data.timestamp) *
                          kSecondToMillisecond;

  // Build pose logging string for debug display.
  std::stringstream string_stream;
  string_stream.setf(std::ios_base::fixed, std::ios_base::floatfield);
  string_stream.precision(3);
  string_stream << "status: "
                << getStatusStringFromStatusCode(cur_pose_data.status_code)
                << ", count: " << pose_status_count
                << ", delta time(ms): " << pose_frame_delta_time
                << ", position(m): [" << cur_pose_data.translation[0] << ", "
                << cur_pose_data.translation[1] << ", "
                << cur_pose_data.translation[2] << "]"
                << ", orientation: [" << cur_pose_data.orientation[0] << ", "
                << cur_pose_data.orientation[1] << ", "
                << cur_pose_data.orientation[2] << ", "
                << cur_pose_data.orientation[3] << "]";
  pose_string = string_stream.str();

  // Update device with respect to  start of service frame transformation.
  // Note: this is the pose transformation for pose frame.
  d_2_ss_mat_motion = glm::translate(glm::mat4(1.0f), tango_position) *
                      glm::mat4_cast(tango_rotation);

  // Store current pose data to previous.
  prev_pose_data = cur_pose_data;
  is_pose_dirty = false;
  pthread_mutex_unlock(&pose_mutex);
}

// Update XYZij data. This function will be called only when the XYZ_ij
// data is changed (dirty). This function is being called through the
// GL rendering thread (See tango_pointcloud.cpp, RenderFrame()).
//
// This will off load some computation inside the onXYZ_ijAvailable()
// callback function. Heavy computation inside callback will block the whole
// Tango Service callback thread, so migrating heavy computation to other
// thread is suggested.
void TangoData::UpdateXYZijData() {
  pthread_mutex_lock(&xyzij_mutex);

  // Calculating average depth for debug display.
  float total_z = 0.0f;
  for (uint32_t i = 0; i < depth_buffer_size; ++i) {
    // The memory layout is x,y,z,x,y,z. We are accumulating
    // all of the z value.
    total_z += depth_buffer[i * 3 + 2];
  }
  if (depth_buffer_size != 0) {
    depth_average_length = total_z / static_cast<float>(depth_buffer_size);
  }

  // Query pose at the depth frame's timestamp.
  // Note: This function is querying pose from pose buffer inside
  // Tango Service. It will pass out the closest pose according to
  // the timestamp passed in.
  TangoCoordinateFramePair pairs;
  pairs.base = TANGO_COORDINATE_FRAME_START_OF_SERVICE;
  pairs.target = TANGO_COORDINATE_FRAME_DEVICE;
  TangoPoseData pose;
  if (TangoService_getPoseAtTime(prev_depth_timestamp, pairs, &pose) !=
      TANGO_SUCCESS) {
    LOGE("TangoService_getPoseAtTime(): Failed");
  }
  glm::vec3 translation =
      glm::vec3(pose.translation[0], pose.translation[1], pose.translation[2]);
  glm::quat rotation = glm::quat(pose.orientation[3], pose.orientation[0],
                                 pose.orientation[1], pose.orientation[2]);

  // Update device with respect to  start of service frame transformation.
  // Note: this is the pose transformation for depth frame.
  d_2_ss_mat_depth =
      glm::translate(glm::mat4(1.0f), translation) * glm::mat4_cast(rotation);

  // Reset xyz_ij dirty flag.
  is_xyzij_dirty = false;

  pthread_mutex_unlock(&xyzij_mutex);
}

// Get OpenGL camera with repect to OpenGL world frame transformation.
// Note: motion tracking pose and depth pose are different. Depth updates slower
// than pose update, we always want to use the closest pose to transform
// point cloud to local space to world space.
glm::mat4 TangoData::GetOC2OWMat(bool is_depth_pose) {
  if (is_depth_pose) {
    pthread_mutex_lock(&xyzij_mutex);
    glm::mat4 temp = d_2_ss_mat_depth;
    pthread_mutex_unlock(&xyzij_mutex);
    return ss_2_ow_mat * temp * glm::inverse(d_2_imu_mat) * c_2_imu_mat *
           oc_2_c_mat;
  } else {
    pthread_mutex_lock(&pose_mutex);
    glm::mat4 temp = d_2_ss_mat_motion;
    pthread_mutex_unlock(&pose_mutex);
    return ss_2_ow_mat * temp * glm::inverse(d_2_imu_mat) * c_2_imu_mat *
           oc_2_c_mat;
  }
}

// Set up extrinsics transformations:
// 1. Device with respect to IMU transformation.
// 2. Color camera with respect to IMU transformation.
// Note: on Yellowstone devices, the color camera is the depth camera.
// so the 'c_2_imu_mat' could also be used for depth point cloud
// transformation.
bool TangoData::SetupExtrinsicsMatrices() {
  TangoPoseData pose_data;
  TangoCoordinateFramePair frame_pair;
  glm::vec3 translation;
  glm::quat rotation;

  // Get device with respect to imu transformation matrix.
  frame_pair.base = TANGO_COORDINATE_FRAME_IMU;
  frame_pair.target = TANGO_COORDINATE_FRAME_DEVICE;
  if (TangoService_getPoseAtTime(0.0, frame_pair, &pose_data) !=
      TANGO_SUCCESS) {
    LOGE("TangoService_getPoseAtTime(): Failed");
    return false;
  }
  translation = glm::vec3(pose_data.translation[0], pose_data.translation[1],
                          pose_data.translation[2]);
  rotation = glm::quat(pose_data.orientation[3], pose_data.orientation[0],
                       pose_data.orientation[1], pose_data.orientation[2]);
  d_2_imu_mat =
      glm::translate(glm::mat4(1.0f), translation) * glm::mat4_cast(rotation);

  // Get color camera with respect to imu transformation matrix.
  frame_pair.base = TANGO_COORDINATE_FRAME_IMU;
  frame_pair.target = TANGO_COORDINATE_FRAME_CAMERA_COLOR;
  if (TangoService_getPoseAtTime(0.0, frame_pair, &pose_data) !=
      TANGO_SUCCESS) {
    LOGE("TangoService_getPoseAtTime(): Failed");
    return false;
  }
  translation = glm::vec3(pose_data.translation[0], pose_data.translation[1],
                          pose_data.translation[2]);
  rotation = glm::quat(pose_data.orientation[3], pose_data.orientation[0],
                       pose_data.orientation[1], pose_data.orientation[2]);
  c_2_imu_mat =
      glm::translate(glm::mat4(1.0f), translation) * glm::mat4_cast(rotation);
  return true;
}

// Clean up.
TangoData::~TangoData() {
  // Free Tango configuration handle.
  if (config_ != nullptr) TangoConfig_free(config_);
  config_ = nullptr;

  delete[] depth_buffer;

  if(frame_external_path   != nullptr){ delete frame_external_path;   frame_external_path = nullptr;}
  if(fisheye_external_path != nullptr){ delete fisheye_external_path; fisheye_external_path = nullptr;}
  if(xyzij_external_path   != nullptr){ delete xyzij_external_path;   xyzij_external_path = nullptr;}
  if(pose_external_path    != nullptr){ delete pose_external_path;    pose_external_path  = nullptr;}
  // TODO: Clean up queues, etc
}

std::string concat_path_and_name(std::string path, std::string name){
    return path+name;
}

int write_cvmat_to_jpeg(string path, string img_name, Mat image){
    std::string absolute_path = concat_path_and_name(path,img_name);
    imwrite(absolute_path.c_str(), image);
    //TODO: Error Code
}

int write_buffer_to_jpeg(string path, string img_name, const TangoImageBuffer * image){
    Mat imageRGBA(image->height, image->width, CV_8UC4, image->data); // TODO: Convert format from RGBA_8888 to BGRA_8888
    write_cvmat_to_jpeg(path,img_name,imageRGBA);
    imageRGBA.release();
    //TODO: Error Code
}
