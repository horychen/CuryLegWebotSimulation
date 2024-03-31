#include <stdio.h>
#include <math.h>

#define CONTROLLER_TYPE 0
#define PI 3.14159265
#define BEZIER_ORDER 4
#define BEZIER_TRACE_SIZE 100
#define INIT_THETA1 0.0
#define INIT_THETA2 0.0

typedef struct
{
    double L1;
    double L2;
    double theta1, theta2;
    double T;
    double height_limit[2];
    double C[BEZIER_ORDER][2];
    int order;
    double bezier_trace[BEZIER_TRACE_SIZE + 1][2];
} CURYCONTROLLER;

CURYCONTROLLER curycontroller = {
    .L1 = 0.4,
    .L2 = 0.39495,
    .theta1 = INIT_THETA1,
    .theta2 = INIT_THETA2,
    .T = 1,
    .height_limit = {0.2, 0.7},
    .C = {{0.0, 0.0}, {0.1, 1.0}, {0.21, 0.93}, {1.0, 1.0}},
    .order = BEZIER_ORDER};

void calc_theta_from_height(double height)
{
    curycontroller.theta1 = acos((pow(curycontroller.L1, 2) + pow(height, 2) - pow(curycontroller.L2, 2)) / (2 * curycontroller.L1 * height));
    curycontroller.theta2 = acos((pow(curycontroller.L2, 2) + pow(height, 2) - pow(curycontroller.L1, 2)) / (2 * curycontroller.L2 * height));
}

void linear_controller(double t)
{
    double height = 0.0;
    int period = (int)(t / curycontroller.T);
    t = t - period * curycontroller.T;
    if (period % 2 == 0)
    {
        height = curycontroller.height_limit[0] + (curycontroller.height_limit[1] - curycontroller.height_limit[0]) * t / curycontroller.T;
    }
    else
    {
        height = curycontroller.height_limit[1] - (curycontroller.height_limit[1] - curycontroller.height_limit[0]) * t / curycontroller.T;
    }
    calc_theta_from_height(height);
}

void sinusoidal_controller(double t)
{
    double height = (curycontroller.height_limit[1] - curycontroller.height_limit[0]) / 2 * sin(2 * PI * t / curycontroller.T) + (curycontroller.height_limit[1] + curycontroller.height_limit[0]) / 2;
    calc_theta_from_height(height);
}

void get_bezier_points()
{
    double t;
    for (int i = 0; i <= BEZIER_TRACE_SIZE; i++)
    {
        t = (double)i / BEZIER_TRACE_SIZE;
        curycontroller.bezier_trace[i][0] = 0.0;
        curycontroller.bezier_trace[i][1] = 0.0;
        for (int j = 0; j < curycontroller.order; j++)
        {
            curycontroller.bezier_trace[i][0] += curycontroller.C[j][0] * pow(1 - t, curycontroller.order - 1 - j) * pow(t, j);
            curycontroller.bezier_trace[i][1] += curycontroller.C[j][1] * pow(1 - t, curycontroller.order - 1 - j) * pow(t, j);
        }
    }
    for (int i = 0; i < BEZIER_TRACE_SIZE; i++)
    {
        for (int j = i + 1; j < BEZIER_TRACE_SIZE; j++)
        {
            if (curycontroller.bezier_trace[i][0] > curycontroller.bezier_trace[j][0])
            {
                double temp[2];
                temp[0] = curycontroller.bezier_trace[i][0];
                temp[1] = curycontroller.bezier_trace[i][1];
                curycontroller.bezier_trace[i][0] = curycontroller.bezier_trace[j][0];
                curycontroller.bezier_trace[i][1] = curycontroller.bezier_trace[j][1];
                curycontroller.bezier_trace[j][0] = temp[0];
                curycontroller.bezier_trace[j][1] = temp[1];
            }
        }
    }
}

void bezier_linear_interpoolation(double t)
{
    for (int i = 0; i < BEZIER_TRACE_SIZE; i++)
    {
        if (curycontroller.bezier_trace[i][0] <= t && t <= curycontroller.bezier_trace[i + 1][0])
        {
            double height = (curycontroller.bezier_trace[i][1] + (curycontroller.bezier_trace[i + 1][1] - curycontroller.bezier_trace[i][1]) * (t - curycontroller.bezier_trace[i][0]) / (curycontroller.bezier_trace[i + 1][0] - curycontroller.bezier_trace[i][0])) * (curycontroller.height_limit[1] - curycontroller.height_limit[0]) + curycontroller.height_limit[0];
            calc_theta_from_height(height);
            return;
        }
    }
}

void bezier_controller(double t)
{
    double height = 0.0;
    int period = (int)(t / curycontroller.T);
    t = (t - period * curycontroller.T)/curycontroller.T;
    if (period % 2 == 0)
    {
        bezier_linear_interpoolation(t);
    }
    else
    {
        height = curycontroller.height_limit[1] - (curycontroller.height_limit[1] - curycontroller.height_limit[0]) * t / curycontroller.T;
        calc_theta_from_height(height);
    }
}

void analoge_controller(double height)
{
    calc_theta_from_height(height);
}

void main()
{
    double t = 0.0;
    get_bezier_points();
    FILE *fp1;
    FILE *fp2;
    FILE *fp3;
    fp1 = fopen("linear.txt", "w");
    fp2 = fopen("sinusoidal.txt", "w");
    fp3 = fopen("bezier.txt", "w");
    if (fp1 == NULL)
    {
        printf("Error opening linear file!\n");
        return;
    }
    if (fp2 == NULL)
    {
        printf("Error opening sinusoidal file!\n");
        return;
    }
    if (fp3 == NULL)
    {
        printf("Error opening bezier file!\n");
        return;
    }
    for (int _ = 0; _ < 3; _++){
        for (int i = 0; i <= 1000; i++)
        {
            switch (_)
            {
            case 0:
                linear_controller(t);
                fprintf(fp1, "%f %f %f\n", t, curycontroller.theta1, curycontroller.theta2+curycontroller.theta1);
                break;
            case 1:
                sinusoidal_controller(t);
                fprintf(fp2, "%f %f %f\n", t, curycontroller.theta1, curycontroller.theta2+curycontroller.theta1);
                break;
            case 2:
                bezier_controller(t);
                fprintf(fp3, "%f %f %f\n", t, curycontroller.theta1, curycontroller.theta2+curycontroller.theta1);
                break;
            default:
                break;
            }
            t += 0.006;
        }
    }
}