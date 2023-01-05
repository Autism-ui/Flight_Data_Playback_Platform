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

#include <webots/robot.h>
#include <webots/supervisor.h>
#include <webots/keyboard.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
/*
 * You may want to add macros here.
 */
#define TIME_STEP 64

#define VIEWFOLLOW ON                                       // Follow View can set ON or OFF
#define CSV_PATH ".\\data.csv" // File Path Set
#define DATA_NUM 1826                                         // data total num
#define scale 0.05                                            // plotting scale set

#define ON true
#define OFF false

typedef struct
{
    double q1;
    double q2;
    double q3;
    double q4;
} QUATERNION_t;

typedef struct
{
    double x;
    double y;
    double z;
    double angle;
} AXISANGLE_t;

QUATERNION_t quaternion[DATA_NUM];
double H[DATA_NUM];
AXISANGLE_t axis_angle;
/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
void quaternion_axis(QUATERNION_t *quaternion); // 将四元数转化为轴角
void automode(int ms);                          // 自动模式
void read_csv(void);                            // 读取*.csv文件
char *get_field(char *line, int num);           // 获取csv文件的数据
char *remove_quoted(char *str);

static int index = 0;     // 离散点下标
static int manual = 0;    // 手动控制标志位
static int automatic = 0; // 自动标志位

int main(int argc, char **argv)
{
    /* necessary to initialize webots stuff */
    wb_robot_init();
    // Load *.csv data
    read_csv();
    /* World and Object Initialize Start */
    // Get Node's DEF
    WbNodeRef node = wb_supervisor_node_get_from_def("AIR");
    WbNodeRef floor_node = wb_supervisor_node_get_from_def("FLOOR");
    WbNodeRef view_node = wb_supervisor_node_get_from_def("VIEW");

    // Get Node's attribute name
    WbFieldRef floor_pos = wb_supervisor_node_get_field(floor_node, "translation");
    WbFieldRef pos = wb_supervisor_node_get_field(node, "translation");
    WbFieldRef rot = wb_supervisor_node_get_field(node, "rotation");

    double data_pos[3] = {0, 0, H[0] * scale};
    double data_rot[4] = {0, 1, 0, 0};
    double floor_init[3] = {0, 0, H[0] * scale - 0.06};

    wb_supervisor_field_set_sf_vec3f(floor_pos, floor_init);
    wb_supervisor_field_set_sf_vec3f(pos, data_pos);

#if VIEWFOLLOW == ON
    WbFieldRef view_pos = wb_supervisor_node_get_field(view_node, "position");
    WbFieldRef view_rot = wb_supervisor_node_get_field(view_node, "orientation");
    double viewpos_follow[3] = {-2.09, 0.00201, 0.479 + H[0] * scale};
    wb_supervisor_field_set_sf_vec3f(view_pos, viewpos_follow);

    const double *Air_Rot = wb_supervisor_field_get_sf_rotation(rot);
    double viewrot_follow[4] = {Air_Rot[0] - 0.499, Air_Rot[1] + 0.499, Air_Rot[2] - 0.292, 4.436 + Air_Rot[3]};
    wb_supervisor_field_set_sf_rotation(view_rot, viewrot_follow);
#else
    WbFieldRef follow_Type = wb_supervisor_node_get_field(view_node, "followType");
    WbFieldRef follow_Name = wb_supervisor_node_get_field(view_node, "follow");
    wb_supervisor_field_set_sf_string(follow_Name, "Plane");
    wb_supervisor_field_set_sf_string(follow_Type, "Pan and Tilt Shot");
#endif
    /* World and Object Initialize End */
    int timestep = (int)wb_robot_get_basic_time_step();
    wb_keyboard_enable(timestep);
    /* main loop
     * Perform simulation steps of TIME_STEP milliseconds
     * and leave the loop when the simulation is over
     */
    while (wb_robot_step(TIME_STEP) != -1)
    {
        // Gain keyboard value
        int key = wb_keyboard_get_key();
        while (key > 0)
        {
            switch (key)
            {
            case (WB_KEYBOARD_CONTROL + WB_KEYBOARD_UP):
                automatic = 1;
                if (automatic && (index < DATA_NUM - 1))
                {
                    printf("Auto Mode Start\n");
                    automode(3);
                }
                else
                {
                    printf("Can't start Auto Mode\n");
                }
                break;
            case WB_KEYBOARD_DOWN:
                index = 0;
                quaternion_axis(&quaternion[index]);
                data_pos[2] = H[index] * scale;

                data_rot[0] = axis_angle.x;
                data_rot[1] = axis_angle.y;
                data_rot[2] = axis_angle.z;
                data_rot[3] = axis_angle.angle;
#if VIEWFOLLOW == ON
                viewpos_follow[2] = 0.479 + H[index] * scale;
                wb_supervisor_field_set_sf_vec3f(view_pos, viewpos_follow);
#endif
                wb_supervisor_field_set_sf_vec3f(pos, data_pos);
                wb_supervisor_field_set_sf_rotation(rot, data_rot);
                printf("Altitude Reset\n");
                break;
            case WB_KEYBOARD_LEFT:
                --index;
                if (index < 0)
                {
                    index = 0;
                    printf("index is lowest,do not press left\n");
                }
                quaternion_axis(&quaternion[index]);
                data_pos[2] = H[index] * scale;

                data_rot[0] = axis_angle.x;
                data_rot[1] = axis_angle.y;
                data_rot[2] = axis_angle.z;
                data_rot[3] = axis_angle.angle;
#if VIEWFOLLOW == ON
                viewpos_follow[2] = 0.479 + H[index] * scale;
                wb_supervisor_field_set_sf_vec3f(view_pos, viewpos_follow);
#endif
                wb_supervisor_field_set_sf_vec3f(pos, data_pos);
                wb_supervisor_field_set_sf_rotation(rot, data_rot);
                printf("H = %f\n", H[index]);
                break;
            case WB_KEYBOARD_RIGHT:
                ++index;
                if (index > DATA_NUM - 1)
                {
                    index = DATA_NUM - 1;
                    printf("index is highest,do not press right\n");
                }
                quaternion_axis(&quaternion[index]);
                data_pos[2] = H[index] * scale;

                data_rot[0] = axis_angle.x;
                data_rot[1] = axis_angle.y;
                data_rot[2] = axis_angle.z;
                data_rot[3] = axis_angle.angle;
#if VIEWFOLLOW == ON
                viewpos_follow[2] = 0.479 + H[index] * scale;
                wb_supervisor_field_set_sf_vec3f(view_pos, viewpos_follow);
#endif
                wb_supervisor_field_set_sf_vec3f(pos, data_pos);
                wb_supervisor_field_set_sf_rotation(rot, data_rot);
                printf("H = %f\n", H[index]);
                break;
            }
            key = wb_keyboard_get_key();
        }
    };
    /* This is necessary to cleanup webots resources */
    wb_robot_cleanup();

    return 0;
}

void read_csv()
{
    FILE *fp = fopen(CSV_PATH, "r");

    if (fp == NULL)
    {
        fprintf(stderr, "fopen() failed.\n");
        exit(EXIT_FAILURE);
    }

    char row[DATA_NUM];

    static int id = 0;
    char *tmp;
    printf("LOADING DATA....\n");
    while (fgets(row, DATA_NUM, fp) != NULL)
    {
        tmp = get_field(strdup(row), 1);
        H[id] = atof(tmp);

        tmp = get_field(strdup(row), 2);
        quaternion[id].q1 = atof(tmp);

        tmp = get_field(strdup(row), 3);
        quaternion[id].q2 = atof(tmp);

        tmp = get_field(strdup(row), 4);
        quaternion[id].q3 = atof(tmp);

        tmp = get_field(strdup(row), 5);
        quaternion[id].q4 = atof(tmp);
        id++;
        // wb_robot_step(2);
        if (id == DATA_NUM)
        {
            printf("Load Data Complete.Total num is %d\n", id);
            break;
        }
    }
    fclose(fp);
}

char *get_field(char *line, int num)
{
    char *tok;
    tok = strtok(line, ",");
    for (int i = 1; i != num; i++)
    {
        tok = strtok(NULL, ",");
    }
    char *result = remove_quoted(tok);
    return result;
}

char *remove_quoted(char *str)
{
    int len = strlen(str);
    char *result = malloc(len + 1);
    int idx = 0;
    for (int i = 0; i < len; i++)
    {
        if (str[i] != '\"')
        {
            result[idx] = str[i];
            idx++;
        }
    }
    result[idx] = '\0';
    return result;
}

void quaternion_axis(QUATERNION_t *quaternion)
{
    double qw = quaternion->q1;
    double qx = quaternion->q2;
    double qy = quaternion->q3;
    double qz = quaternion->q4;
    axis_angle.x = qx / sqrt(1 - qw * qw);
    axis_angle.y = qy / sqrt(1 - qw * qw);
    axis_angle.z = qz / sqrt(1 - qw * qw);
    axis_angle.angle = 2 * acos(qw);
}

void automode(int ms)
{
    WbNodeRef node = wb_supervisor_node_get_from_def("AIR");
    WbNodeRef view_node = wb_supervisor_node_get_from_def("VIEW");

    WbFieldRef pos = wb_supervisor_node_get_field(node, "translation");
    WbFieldRef rot = wb_supervisor_node_get_field(node, "rotation");

    double data_pos[3] = {0, 0, H[0]};
    double data_rot[4] = {0, 1, 0, 0};

#if VIEWFOLLOW == ON
    WbFieldRef view_pos = wb_supervisor_node_get_field(view_node, "position");
    double viewpos_follow[3] = {-2.09, 0.00201, 0.479 + H[0] * scale};
#endif

    while ((wb_robot_step(ms) != -1) && automatic == 1)
    {
        quaternion_axis(&quaternion[index]);
        data_pos[2] = H[index] * scale;

        data_rot[0] = axis_angle.x;
        data_rot[1] = axis_angle.y;
        data_rot[2] = axis_angle.z;
        data_rot[3] = axis_angle.angle;
#if VIEWFOLLOW == ON
        viewpos_follow[2] = 0.479 + H[index] * scale;
        wb_supervisor_field_set_sf_vec3f(view_pos, viewpos_follow);
#endif
        wb_supervisor_field_set_sf_vec3f(pos, data_pos);
        wb_supervisor_field_set_sf_rotation(rot, data_rot);
        ++index;
        printf("H = %f\n", H[index]);
        int key = wb_keyboard_get_key();
        while (key > 0)
        {
            switch (key)
            {
            case (WB_KEYBOARD_CONTROL + WB_KEYBOARD_DOWN):
                automatic = 0;
                manual = 1;
                printf("Auto Mode is Pause\n");
                break;
            }
            key = wb_keyboard_get_key();
        }
        if (index == DATA_NUM - 1)
        {
            automatic = 0;
            printf("Auto Mode is Over\n");
            break;
        }
    }
}