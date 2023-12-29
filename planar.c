#include <math.h>
#define PI 3.14159265358
#define DTR PI / 180.0
#define RTD 180.0 / PI
#define L1 0.3
#define L2 0.2



float q1, q2;
float dq1, dq2;
float x_res, y_res;
float x_res_init, y_res_init;

float dutyCmd1;
float dutyCmd2;

float objx=0.3;
float objy=0.5;

float error_x = 0, error_y = 0;
float errorOld_x = 0, errorOld_y = 0;
float integralError_x = 0;
float integralError_y = 0;
float u_x = 0;
float u_y = 0;
float Kp = 70;
float Ki = 40;
float Kd = 1;
float K = 1;
float u_teta1 = 0, u_teta2 = 0;
float v1, v2;

float t = 0;
float dt = 0.017;

float x_cmd, y_cmd;
float r_tra = 0.15;
float tra_x = -0.15, tra_y = 0.3;
float center_x = 0.15, center_y = 0.3;

float time_traj = 2.5;
int linemode = 0;

float access_q1 = 0;
float access_q2 = 0;

void init_robot()
{
    q1 = 0.0 * DTR;
    q2 = 5.0 * DTR;
}

void forward_kinematic(float teta1, float teta2, float l1, float l2)
{
    x_res = l1 * cos(teta1) + l2 * cos(teta1 + teta2);
    y_res = l1 * sin(teta1) + l2 * sin(teta1 + teta2);
}

void inv_jacobian(float teta1, float teta2, float l1, float l2, float dx, float dy)
{
    float c11, c12, c21, c22;
    static float den, den_old;
    den = l2 * sin(teta1 + teta2) * (l2 * cos(teta1 + teta2) + l1 * cos(teta1)) - l2 * cos(teta1 + teta2) * (l2 * sin(teta1 + teta2) + l1 * sin(teta1));
    c11 = l2 * cos(teta1 + teta2) / den;
    c12 = l2 * sin(teta1 + teta2) / den;
    c21 = -(l2 * cos(teta1 + teta2) + l1 * cos(teta1)) / den;
    c22 = -(l2 * sin(teta1 + teta2) + l1 * sin(teta1)) / den;
    u_teta1 = c11 * dx + c12 * dy;
    u_teta2 = c21 * dx + c22 * dy;
}

void trajectory_circle(float t)
{
    tra_x = cos(t * 72 * DTR) * r_tra + center_x;
    tra_y = sin(t * 72 * DTR) * r_tra + center_y;
}

void trajectory_line(float t)
{
    if (t <= dt)
    {
        x_res_init = x_res;
        y_res_init = y_res;
        linemode = (linemode == 1) ? 0 : 1;
        if (linemode == 0)
        {
            x_cmd = x_res - 0.05;
            y_cmd = y_res + 0.2;
        }
        else
        {
            x_cmd = x_res + 0.05;
            y_cmd = y_res - 0.2;
        }
    }
    tra_x = (x_cmd - x_res_init) * t / time_traj + x_res_init;
    tra_y = (y_cmd - y_res_init) * t / time_traj + y_res_init;
}

void controlX(void)
{
    errorOld_x = error_x;
    error_x = tra_x - x_res;
    integralError_x += error_x * dt;
    u_x = Kp * error_x + (Kd / dt) * (error_x - errorOld_x) + Ki * integralError_x;
}

void controlY(void)
{
    errorOld_y = error_y;
    error_y = tra_y - y_res;
    integralError_y += error_y * dt;
    u_y = Kp * error_y + (Kd / dt) * (error_y - errorOld_y) + Ki * integralError_y;
}

void save_data()
{
    FILE *fptr = fopen("datalog.csv", "a");
    if (fptr == NULL)
    {
        printf("Error!");
        exit(1);
    }

    fprintf(fptr, "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n", t, y_cmd, y_res, x_cmd, x_res, (q1 * RTD), (q2 * RTD), dq1 * 150, dq2 * 150);
    fclose(fptr);
}

void animate(int k)
{ 
    // q1 = access_q1;
    // q2 = access_q2;

    forward_kinematic(q1, q2, L1, L2);
    trajectory_line(time_traj * (t / time_traj - trunc(t / time_traj)));
    controlX();
    controlY();
    inv_jacobian(q1, q2, L1, L2, u_x, u_y);
    save_data();
    v1 = K * u_teta1;
    v2 = K * u_teta2;
    dq1 = dq1 + (2.083 * v1 - 2.71 * dq1) * dt;
    dq2 = dq2 + (2.083 * v2 - 2.71 * dq2) * dt;
    q1 = q1 + (dq1 * dt);
    q2 = q2 + (dq2 * dt);
    t += dt;
    printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n", t, y_cmd, y_res, x_cmd, x_res, (q1 * RTD), (q2 * RTD), dq1 * 150, dq2 * 150);
}
