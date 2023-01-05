/*
 * File:          record_control.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/motor.h>, etc.
 */

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <webots/keyboard.h>
#include <webots/robot.h>
#include <webots/supervisor.h>
/*
 * You may want to add macros here.
 */
#define TIME_STEP 32

#define CSV_PATH "..\\..\\data.csv" // File Path Set
#define scale 1                     // plotting scale set

typedef struct {
  double q1;
  double q2;
  double q3;
  double q4;
} QUATERNION_t;

typedef struct {
  double x;
  double y;
  double z;
  double angle;
} AXISANGLE_t;

typedef struct {
  float Pitch;
  float Roll;
  float Yaw;
} EULER_t;

QUATERNION_t *quaternion;
double *H;
double *t;
AXISANGLE_t axis_angle;
EULER_t euler;
/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
void quaternion_axis(QUATERNION_t *quaternion); // 将四元数转化为轴角
void quaternion_euler(QUATERNION_t *quaternion);
void automode(int ms); // 自动模式
void calcrow_csv(void);
void OperationShow(void);
void LOG_HMI(void);
void read_csv(void);                  // 读取*.csv文件
char *get_field(char *line, int num); // 获取csv文件的数据
char *remove_quoted(char *str);

static int index = 0;     // 离散点下标
static int manual = 0;    // 手动控制标志位
static int automatic = 0; // 自动标志位
static int vision_mode = 0; // 视角模式选择：0.定视角模式 1.自由角模式
static int DATA_NUM = 0;                   // 数据行数
static double altimetric_compensation = 0; // 高度补偿
int main(int argc, char **argv) {
  /* necessary to initialize webots stuff */
  LOG_HMI();
  wb_robot_init();
  // Load *.csv data
  calcrow_csv();
  read_csv();
  OperationShow();
  /* World and Object Initialize Start */
  // Get Node's DEF
  WbNodeRef node = wb_supervisor_node_get_from_def("AIR");
  WbNodeRef floor_node = wb_supervisor_node_get_from_def("FLOOR");
  WbNodeRef view_node = wb_supervisor_node_get_from_def("VIEW");

  // Get Node's attribute name
  WbFieldRef floor_pos =
      wb_supervisor_node_get_field(floor_node, "translation");
  WbFieldRef pos = wb_supervisor_node_get_field(node, "translation");
  WbFieldRef rot = wb_supervisor_node_get_field(node, "rotation");

  altimetric_compensation = H[0] < 0 ? abs(H[0] * scale) : (-H[0] * scale);
  double data_pos[3] = {0, 0, H[0] * scale + altimetric_compensation};
  double data_rot[4] = {0, 1, 0, 0};
  double floor_init[3] = {0, 0, H[0] * scale + altimetric_compensation - 0.06};

  wb_supervisor_field_set_sf_vec3f(floor_pos, floor_init);
  wb_supervisor_field_set_sf_vec3f(pos, data_pos);

  // Vision-FollowMode Init
  WbFieldRef view_pos = wb_supervisor_node_get_field(view_node, "position");
  WbFieldRef view_rot = wb_supervisor_node_get_field(view_node, "orientation");
  double viewpos_follow[3] = {-2.09, 0.00201,
                              0.479 + H[0] * scale + altimetric_compensation};
  double viewrot_follow[4] = {-0.528, 0.528, 0.664, 4.31};

  // 只有在初始化的时候才需要用到这个定位
  // const double *Air_Rot = wb_supervisor_field_get_sf_rotation(rot);
  // double viewrot_follow[4] = {Air_Rot[0] - 0.499, Air_Rot[1] + 0.499,
  // Air_Rot[2] - 0.292, 4.436 + Air_Rot[3]};

  wb_supervisor_field_set_sf_vec3f(view_pos, viewpos_follow);
  wb_supervisor_field_set_sf_rotation(view_rot, viewrot_follow);

  // Vision-MultiVisionMode Init
  WbFieldRef follow_Type =
      wb_supervisor_node_get_field(view_node, "followType");
  WbFieldRef follow_Name = wb_supervisor_node_get_field(view_node, "follow");
  wb_supervisor_field_set_sf_string(follow_Name, "Plane");
  wb_supervisor_field_set_sf_string(follow_Type, "None");

  /* World and Object Initialize End */
  int timestep = (int)wb_robot_get_basic_time_step();
  char str_H[20], str_T[20];
  memset(str_H, 0, sizeof(str_H));
  memset(str_T, 0, sizeof(str_T));
  wb_keyboard_enable(timestep);
  wb_supervisor_set_label(1, "t:", 0.7, 0.15, 0.1, 0x00ff, 0.1, "Arial");
  wb_supervisor_set_label(3, "s", 0.9, 0.15, 0.1, 0xff8800, 0.1, "Arial");
  wb_supervisor_set_label(4, "H:", 0.7, 0.2, 0.1, 0x00ff, 0.1, "Arial");
  wb_supervisor_set_label(6, "m", 0.9, 0.2, 0.1, 0xff8800, 0.1, "Arial");
  /* main loop
   * Perform simulation steps of TIME_STEP milliseconds
   * and leave the loop when the simulation is over
   */
  while (wb_robot_step(TIME_STEP) != -1) {
    // Gain keyboard value
    int key = wb_keyboard_get_key();
    while (key > 0) {
      switch (key) {
      case (WB_KEYBOARD_CONTROL + WB_KEYBOARD_UP):
        automatic = 1;
        if (automatic && (index < DATA_NUM - 1)) {
          printf("Auto Mode Start\n");
          automode(3);
        } else {
          printf("Can't start Auto Mode\n");
        }
        break;
      case WB_KEYBOARD_DOWN:
        index = -1;
        // quaternion_axis(&quaternion[index]);
        // H[index] = H[index] > H[0] ? H[index]:H[0];
        // data_pos[2] = H[index] * scale + altimetric_compensation;
        data_pos[2] = H[0] * scale + altimetric_compensation;
        // data_rot[0] = axis_angle.x;
        // data_rot[1] = axis_angle.y;
        // data_rot[2] = axis_angle.z;
        // data_rot[3] = axis_angle.angle;
        data_rot[0] = 0;
        data_rot[1] = 1;
        data_rot[2] = 0;
        data_rot[3] = 0;
        if (vision_mode == 0) {
          viewpos_follow[2] = 0.479 + H[0] * scale + altimetric_compensation;
          wb_supervisor_field_set_sf_vec3f(view_pos, viewpos_follow);
        }
        wb_supervisor_field_set_sf_vec3f(pos, data_pos);
        wb_supervisor_field_set_sf_rotation(rot, data_rot);
        printf("Altitude Reset\n");
        break;
      case WB_KEYBOARD_LEFT:
        --index;
        if (index < 0) {
          index = 0;
          printf("index is lowest,do not press left\n");
        }
        quaternion_axis(&quaternion[index]);
        quaternion_euler(&quaternion[index]);
        H[index] = H[index] > H[0] ? H[index] : H[0];
        data_pos[2] = H[index] * scale + altimetric_compensation;

        data_rot[0] = axis_angle.x;
        data_rot[1] = axis_angle.y;
        data_rot[2] = axis_angle.z;
        data_rot[3] = axis_angle.angle;
        if (vision_mode == 0) {
          viewpos_follow[2] =
              0.479 + H[index] * scale + altimetric_compensation;
          wb_supervisor_field_set_sf_vec3f(view_pos, viewpos_follow);
        }
        wb_supervisor_field_set_sf_vec3f(pos, data_pos);
        wb_supervisor_field_set_sf_rotation(rot, data_rot);
        printf("t=%.3f,H = %f,Pitch = %f,Roll = %f,Yaw = %f\n", t[index],
               H[index], euler.Pitch, euler.Roll, euler.Yaw);
        break;
      case WB_KEYBOARD_RIGHT:
        ++index;
        if (index > DATA_NUM - 1) {
          index = DATA_NUM - 1;
          printf("index is highest,do not press right\n");
        }
        quaternion_axis(&quaternion[index]);
        quaternion_euler(&quaternion[index]);
        H[index] = H[index] > H[0] ? H[index] : H[0];
        data_pos[2] = H[index] * scale + altimetric_compensation;

        data_rot[0] = axis_angle.x;
        data_rot[1] = axis_angle.y;
        data_rot[2] = axis_angle.z;
        data_rot[3] = axis_angle.angle;
        if (vision_mode == 0) {
          viewpos_follow[2] =
              0.479 + H[index] * scale + altimetric_compensation;
          wb_supervisor_field_set_sf_vec3f(view_pos, viewpos_follow);
        }
        wb_supervisor_field_set_sf_vec3f(pos, data_pos);
        wb_supervisor_field_set_sf_rotation(rot, data_rot);
        printf("t=%.3f,H = %f,Pitch = %f,Roll = %f,Yaw = %f\n", t[index],
               H[index], euler.Pitch, euler.Roll, euler.Yaw);
        break;
      case (WB_KEYBOARD_ALT + WB_KEYBOARD_UP):
        vision_mode = 1;
        wb_supervisor_field_set_sf_string(follow_Type, "Tracking Shot");
        printf("Mode is change to Multiply View Mode\n");
        break;
      case (WB_KEYBOARD_ALT + WB_KEYBOARD_DOWN):
        vision_mode = 0;
        wb_supervisor_field_set_sf_rotation(view_rot, viewrot_follow);
        H[index] = H[index] > H[0] ? H[index] : H[0];
        viewpos_follow[2] = 0.479 + H[index] * scale + altimetric_compensation;
        wb_supervisor_field_set_sf_vec3f(view_pos, viewpos_follow);
        wb_supervisor_field_set_sf_string(follow_Type, "None");
        printf("Mode is change to FixedView Mode\n");
        break;
      }
      key = wb_keyboard_get_key();
    }
    sprintf(str_H, "%.2f", H[index]);
    sprintf(str_T, "%.3f", t[index]);
    wb_supervisor_set_label(2, str_T, 0.75, 0.15, 0.1, 0xff8800, 0.1, "Arial");
    wb_supervisor_set_label(5, str_H, 0.75, 0.2, 0.1, 0xff8800, 0.1, "Arial");
  };
  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}

void read_csv() {
  FILE *fp = fopen(CSV_PATH, "r");

  if (fp == NULL) {
    fprintf(stderr, "fopen() failed.\n");
    exit(EXIT_FAILURE);
  }

  char row[DATA_NUM];

  static int id = 0;
  char *tmp;
  printf("LOADING DATA....\n");

  fgets(row, DATA_NUM, fp);
  char *header_1 = get_field(strdup(row), 1);
  char *header_2 = get_field(strdup(row), 2);
  char *header_3 = get_field(strdup(row), 3);
  char *header_4 = get_field(strdup(row), 4);
  char *header_5 = get_field(strdup(row), 5);
  char *header_6 = get_field(strdup(row), 6);
  printf("Data import sequence:%s->%s->%s->%s->%s->%s\n", header_1, header_2,
         header_3, header_4, header_5, header_6);

  while (fgets(row, DATA_NUM, fp) != NULL) {
    tmp = get_field(strdup(row), 1);
    t[id] = atof(tmp) / 1000;

    tmp = get_field(strdup(row), 2);
    H[id] = atof(tmp);

    tmp = get_field(strdup(row), 3);
    quaternion[id].q1 = atof(tmp);

    tmp = get_field(strdup(row), 4);
    quaternion[id].q2 = atof(tmp);

    tmp = get_field(strdup(row), 5);
    quaternion[id].q3 = atof(tmp);

    tmp = get_field(strdup(row), 6);
    quaternion[id].q4 = atof(tmp);
    id++;
    if (id == DATA_NUM) {
      printf("Load Data Complete.\nTotal num is %d\n", id);
      break;
    }
  }
  fclose(fp);
}

char *get_field(char *line, int num) {
  char *tok;
  tok = strtok(line, ",");
  for (int i = 1; i != num; i++) {
    tok = strtok(NULL, ",");
  }
  char *result = remove_quoted(tok);
  return result;
}

char *remove_quoted(char *str) {
  int len = strlen(str);
  char *result = malloc(len + 1);
  int idx = 0;
  for (int i = 0; i < len; i++) {
    if (str[i] != '\"') {
      result[idx] = str[i];
      idx++;
    }
  }
  result[idx] = '\0';
  return result;
}

void quaternion_axis(QUATERNION_t *quaternion) {
  double qw = quaternion->q1;
  double qx = quaternion->q2;
  double qy = quaternion->q3;
  double qz = quaternion->q4;
  axis_angle.x = qx / sqrt(1 - qw * qw);
  axis_angle.y = qy / sqrt(1 - qw * qw);
  axis_angle.z = qz / sqrt(1 - qw * qw);
  axis_angle.angle = 2 * acos(qw);
}

void quaternion_euler(QUATERNION_t *quaternion) {
  double q0 = quaternion->q1;
  double q1 = quaternion->q2;
  double q2 = quaternion->q3;
  double q3 = quaternion->q4;
  float degrees = (180.0f / (float)M_PI);
  euler.Pitch = (float)(asin(-2 * q1 * q3 + 2 * q0 * q2) * degrees);
  euler.Roll =
      (float)(atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) *
              degrees);
  euler.Yaw = (float)(atan2(2 * (q1 * q2 + q0 * q3),
                            q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) *
                      degrees);
}

void automode(int ms) {
  WbNodeRef node = wb_supervisor_node_get_from_def("AIR");
  WbNodeRef view_node = wb_supervisor_node_get_from_def("VIEW");

  WbFieldRef pos = wb_supervisor_node_get_field(node, "translation");
  WbFieldRef rot = wb_supervisor_node_get_field(node, "rotation");

  H[index] = H[index] > H[0] ? H[index] : H[0];
  double data_pos[3] = {0, 0, H[0] * scale + altimetric_compensation};
  double data_rot[4] = {0, 1, 0, 0};

  // Vision-FollowMode Init
  WbFieldRef view_pos = wb_supervisor_node_get_field(view_node, "position");
  WbFieldRef view_rot = wb_supervisor_node_get_field(view_node, "orientation");
  double viewpos_follow[3] = {-2.09, 0.00201,
                              0.479 + H[0] * scale + altimetric_compensation};
  double viewrot_follow[4] = {-0.528, 0.528, 0.664, 4.31};

  // Vision-MultiVisionMode Init
  WbFieldRef follow_Type =
      wb_supervisor_node_get_field(view_node, "followType");
  WbFieldRef follow_Name = wb_supervisor_node_get_field(view_node, "follow");
  wb_supervisor_field_set_sf_string(follow_Name, "Plane");

  char str_H[10], str_T[10];
  memset(str_H, 0, sizeof(str_H));
  memset(str_T, 0, sizeof(str_T));
  while ((wb_robot_step(ms) != -1) && automatic == 1) {
    quaternion_axis(&quaternion[index]);
    quaternion_euler(&quaternion[index]);
    H[index] = H[index] > H[0] ? H[index] : H[0];
    data_pos[2] = H[index] * scale + altimetric_compensation;
    data_rot[0] = axis_angle.x;
    data_rot[1] = axis_angle.y;
    data_rot[2] = axis_angle.z;
    data_rot[3] = axis_angle.angle;

    if (vision_mode == 0) {
      viewpos_follow[2] = 0.479 + H[index] * scale + altimetric_compensation;
      wb_supervisor_field_set_sf_vec3f(view_pos, viewpos_follow);
    }
    wb_supervisor_field_set_sf_vec3f(pos, data_pos);
    wb_supervisor_field_set_sf_rotation(rot, data_rot);
    printf("t=%.3f,H = %f,Pitch = %f,Roll = %f,Yaw = %f\n", t[index], H[index],
           euler.Pitch, euler.Roll, euler.Yaw);
    ++index;
    int key = wb_keyboard_get_key();
    while (key > 0) {
      switch (key) {
      case (WB_KEYBOARD_CONTROL + WB_KEYBOARD_DOWN):
        automatic = 0;
        manual = 1;
        printf("Auto Mode is Pause\n");
        break;
      case (WB_KEYBOARD_ALT + WB_KEYBOARD_UP):
        vision_mode = 1;
        wb_supervisor_field_set_sf_string(follow_Type, "Tracking Shot");
        printf("Mode is change to Multiply View Mode\n");
        break;
      case (WB_KEYBOARD_ALT + WB_KEYBOARD_DOWN):
        vision_mode = 0;
        wb_supervisor_field_set_sf_rotation(view_rot, viewrot_follow);
        viewpos_follow[2] = 0.479 + H[index] * scale + altimetric_compensation;
        wb_supervisor_field_set_sf_vec3f(view_pos, viewpos_follow);
        wb_supervisor_field_set_sf_string(follow_Type, "None");
        printf("Mode is change to FixedView Mode\n");
        break;
      }
      key = wb_keyboard_get_key();
    }
    if (index == DATA_NUM - 1) {
      automatic = 0;
      printf("Auto Mode is Over\n");
      break;
    }
    sprintf(str_H, "%.2f", H[index]);
    sprintf(str_T, "%.3f", t[index]);
    wb_supervisor_set_label(2, str_T, 0.75, 0.15, 0.1, 0xff8800, 0.1, "Arial");
    wb_supervisor_set_label(5, str_H, 0.75, 0.2, 0.1, 0xff8800, 0.1, "Arial");
  }
}
inline void calcrow_csv() {
  FILE *file_csv;
  file_csv = fopen(CSV_PATH, "r");
  if (file_csv == NULL) {
    fprintf(stderr, "fopen() failed.\n");
    exit(EXIT_FAILURE);
  }
  int flag;
  while (!feof(file_csv)) {
    flag = fgetc(file_csv);
    if (flag == '\n') {
      DATA_NUM++;
    }
  }
  fclose(file_csv);
  DATA_NUM -= 1; // 删除开头行的标识
  quaternion = (QUATERNION_t *)malloc(sizeof(QUATERNION_t) * DATA_NUM);
  H = (double *)malloc(sizeof(double) * DATA_NUM);
  t = (double *)malloc(sizeof(double) * DATA_NUM);
  printf("DATA_NUM is :%d\n", DATA_NUM);
}

void LOG_HMI() {
  printf(
      "/*****************************************************************/\n");
  printf(
      "/*          _____   _            __   _                          */\n");
  printf(
      "/*         |  ___| | |  _   _   / _| (_)  _ __    ___            */\n");
  printf(
      "/*         | |_    | | | | | | | |_  | | | \"__| / _  |           */\n");
  printf(
      "/*         |  _|   | | | |_| | |  _| | | | |    |  __/           */\n");
  printf(
      "/*         |_|     |_| |__, |  |_|   |_| |_|    |___|            */\n");
  printf(
      "/*                      |___/                                    */\n");
  printf(
      "/*---------------------------------------------------------------*/\n");
  printf(
      "/*         This file is mainly used for flight data playback,    */\n");
  printf(
      "/*              please open the Readme for details               */\n");
  printf(
      "/*                                          Author:Autism_Huang  */\n");
  printf(
      "/*****************************************************************/\n");
}
void OperationShow() {
  printf("/************************************/\n");
  printf("/*  ctrl+↑：Auto Record *.csv data  */\n");
  printf("/*  ctrl+↓：Pause auto Record       */\n");
  printf("/*   alt+↑：Multiple view mode      */\n");
  printf("/*   alt+↓：Fixed view mode(Default)*/\n");
  printf("/*     ↓  ：Reset                   */\n");
  printf("/*     ←  ：Play one frame back     */\n");
  printf("/*     →  ：Play one frame forward  */\n");
  printf("/************************************/\n");
}