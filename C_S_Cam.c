#define __USE_MINGW_ANSI_STDIO 1
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#define PI 3.14159265358979323846
const double  O1O6 = 262; //a
const double  O1E = 222; //l
const double OMIGA = 432 * PI /180;

long double LocationOF_X (double theta, long double phi, long double phi0){
        double x = 262 * sin(theta) - 222 * sin(theta + phi + phi0 );
        double a = 262 * sin(theta) - 222 * sin(theta + phi + phi0 );
        return x;
}
//  E点的X坐标

long double LocationOF_Y (double theta, long double phi, long double phi0){
        double y = O1O6 * cos(theta) - O1E * cos(theta + phi + phi0);
        return y;
}
//  E点的Y坐标

long double ConstOF_Phi0 (double r0){
        long double phi0 = acos(((pow(262,2)+pow(222,2)-pow(r0,2))/(2*O1E*O1O6)));
        long double x = (pow(262,2)+pow(222,2)-pow(50,2))/(2*262*222);
        return phi0;
}
//phi0的值

long double Cam_Drive (double theta, double r0){
        long double Cam_Drive = 26.64 * theta + r0;
        return Cam_Drive;
}
//求推程

long double Cam_Return (double theta0,  double theta, double r0){
        long double h = 140 * 0.465;
        long double Cam_Return = h * ((1 + cos(PI * theta / theta0)) / 2)+ r0; 
        return Cam_Return;
}
//求回程
long double Cam_Return_v (double theta0, double theta, double r0) {
        long double h = 140 * 0.442;
        long double Cam_Return_v = (-PI * h * OMIGA * sin(PI * theta / theta0) ) / (2 * theta0) ;
        return Cam_Return_v;
}

long double Cam_Return_a (double theta0, double theta, double r0) {
        long double h = 140 * 0.323;
        long double Cam_Return_a = (- pow(PI,2) * h * pow(OMIGA,2) * cos(PI * theta / theta0))/(2 * pow(theta0 , 2));
        return Cam_Return_a;
}

long double Circle_Location (double radius, double theta){
       double x = radius * cos(theta);
       double y = radius * sin(theta);
       return pow(pow(x,2) + pow(y,2),0.5);
}
//求停止端

//计算位移角
// 计算凸轮上每隔一度理论轮廓线上点的位置
int main(void) {
        double time_all = 0;
        double theta = 0 ; 
        double theta_angel = time_all ;
        double r0 = 150;//基圆
        double theta0 = 143 * PI / 180;
        double circle_x = 262 * cos(theta);
        double circle_y = 262 * sin(theta);
        double rr = 15; //滚子半径
        
        //推程滚子坐标确定
        //CSV file
        FILE *fp = fopen("Cam2.csv","w+");
        if (fp == NULL) {
                fprintf(stderr,"fopen() failed.\n");
                exit(EXIT_FAILURE);
        }
        fprintf(fp,"x,y,theta,x_,y_,alpha,v,a\n");
        
        if (theta_angel <= 140 ) {
                int time1;

                for (time1 = 0; time1 <= 140; ++time1  ) {
                        double theta = time_all * PI / 180;
                        long double theta1 = (time_all + 0.1) * PI / 180;
                        long double drive_long = Cam_Drive ( theta , r0);
                        long double drive_long1 = Cam_Drive(theta1 , r0);
                        long double half_chord = pow((pow(222,2)-pow(((pow(222,2) - pow(drive_long,2) + pow(262,2)) /( 2 * 262)),2)),0.5);
                        long double half_chord1 = pow((pow(222,2)-pow(((pow(222,2) - pow(drive_long1,2) + pow(262,2)) /( 2 * 262)),2)),0.5);
                        long double phi_add_phi0 = asin(half_chord/222);
                        long double phi_add_phi01 = asin(half_chord1/222);
                        long double phi0 = ConstOF_Phi0(r0);
                        long double phi = phi_add_phi0 - phi0;
                        long double phi1 = phi_add_phi01 -phi0;
                        //此时E的坐标
                        long double E_x = LocationOF_X( theta, phi, phi0);
                        long double E_y = LocationOF_Y( theta, phi, phi0);
                        //求 dx dy 的导数
                        long double E_x1 = LocationOF_X(theta1, phi1, phi0);
                        long double E_y1 = LocationOF_Y(theta1, phi1, phi0);
                        long double dx = (E_x1 - E_x)/(0.1 * PI / 180);
                        long double dy = (E_y1 - E_y)/(0.1 * PI / 180);
                        //凸轮实际轮廓线的x y 坐标
                        long double Sin_R = dx / sqrt(pow(dx,2) + pow(dy,2));
                        long double Cos_R = -dy / sqrt(pow(dx,2) + pow(dy,2));
                        long double x_ = E_x - (rr * Cos_R);
                        long double y_ = E_y - (rr * Sin_R); 
                        long double v = 18.49 * OMIGA;
                        double a = 0;
                        // 求压力角
                        //long double o1p = 262 / (1 + 432 * (phi1 - phi));
                        //long double o2p = pow(pow(o1p,2) + pow(222,2) - (2 * o1p * cos(phi + phi0)),0.5 );
                        //long double beta = acos((pow(o2p,2) + pow(222,2) - pow(o1p,2)) / (2 * pow(o2p,2) * 222));
                        // 压力角
                        //long double alpha = 90 - (beta * 180 / PI);
                        long double alpha = atan((1 / tan(phi_add_phi0)) - (222 * (1 - ((phi1 - phi) / 0.1))) / (262 * sin(phi_add_phi0))) * 180 / PI;
                        fprintf(fp,"%Lf,%Lf,%lf,%Lf,%Lf,%Lf,%Lf,%lf \n ",E_x,E_y,time_all,x_,y_,alpha,v,a) ;
                        //printf("alpha = %Lfalpha2 = %Lf \t",alpha,alpha1);
                        printf("d= %Lf\t\t t=%lf",drive_long,time_all);
                        theta_angel ++; 
                        time_all ++;
                        
                }
                printf("\n \n");
        }
        if (theta_angel < 205 && theta_angel > 140) {
                int time;

                for (time = 140 ; time < 205 ; ++time ) {
                        double theta = time_all * PI / 180;
                        long double theta0 = 140 * PI / 180;
                        long double stop_long = Cam_Drive(theta0,r0) ;
                        long double half_chord = pow((pow(222,2)-pow(((pow(222,2) - pow(stop_long,2) + pow(262,2)) /( 2 * 262)),2)),0.5);
                        long double phi_add_phi0 = asin(half_chord/222);
                        long double phi0 = ConstOF_Phi0(r0);
                        long double phi = phi_add_phi0 - phi0;
                        long double E_x_stop = LocationOF_X( theta, phi, phi0);
                        long double E_y_stop = LocationOF_Y( theta, phi, phi0);
                        long double Sin_Ex = E_y_stop / (pow((pow(E_x_stop,2) + pow(E_y_stop,2)),0.5));
                        long double Cos_Ey = E_x_stop / (pow((pow(E_x_stop,2) + pow(E_y_stop,2)),0.5));
                        long double E_x_stop1 = (stop_long - rr) * Cos_Ey;
                        long double E_y_stop1 = (stop_long - rr) * Sin_Ex;
                        //实际轮廓曲线
                        //long double o1p = 262 ;
                        //long double o2p = pow(pow(o1p,2) + pow(222,2) - (2 * o1p * cos(phi + phi0)),0.5 );
                        // double beta = acos((pow(o2p,2) + pow(222,2) - pow(o1p,2)) / (2 * pow(o2p,2) * 222));
                        // 压力角
                        //long double alpha = 90 - (beta * 180 / PI);
                        long double alpha = atan(1 / tan(phi_add_phi0)) * 180 / PI;
                        fprintf(fp,"%Lf,%Lf,%lf,%Lf,%Lf,%Lf,0,0 \n",E_x_stop,E_y_stop,time_all,E_x_stop1,E_y_stop1,alpha);
                        //printf("x=%Lf",E_x_stop1);
                        theta_angel ++;  
                        time_all ++;
                }
                printf("\n \n");
        }
        if (theta_angel < 348 && theta_angel > 205) {
                int time2;

                 for (time2 = 205; time2 < 348 ;++time2 ) {
                        double theta = time_all * PI / 180;
                        double theta1 = (time_all + 0.1) * PI / 180;
                        double theta0 = 143 * PI / 180;
                        double re_theta = (time_all - 205) * PI / 180 ;
                        double re_theta1 = (time_all -205) * PI / 180 ;
                        long double return_long = Cam_Return(theta0 , re_theta , r0);
                        long double return_long1 = Cam_Return(theta0 , re_theta1 , r0);
                        long double half_chord_return = pow((pow(222,2)-pow(((pow(222,2) - pow(return_long,2) + pow(262,2)) /( 2 * 262)),2)),0.5);
                        long double half_chord_return1 = pow((pow(222,2)-pow(((pow(222,2) - pow(return_long1,2) + pow(262,2)) /( 2 * 262)),2)),0.5);
                        double phi_add_phi0_return = asin(half_chord_return/222);
                        double phi_add_phi0_return1 = asin(half_chord_return1/222);
                        long double phi0_re = ConstOF_Phi0(r0);
                        long double phi_re = phi_add_phi0_return - phi0_re;
                        long double phi_re1 = phi_add_phi0_return1 - phi0_re;
                        long double E_x_re = LocationOF_X(theta,phi_re,phi0_re);
                        long double E_y_re = LocationOF_Y(theta,phi_re,phi0_re);
                        long double E_x1 = LocationOF_X(theta1, phi_re1, phi0_re);
                        long double E_y1 = LocationOF_Y(theta1, phi_re1, phi0_re);
                        long double dx = (E_x1 - E_x_re)/(0.1 * PI / 180);
                        long double dy = (E_y1 - E_y_re)/(0.1 * PI / 180);// 求导
                        long double Sin_R = dx / sqrt(pow(dx,2) + pow(dy,2));
                        long double Cos_R = -dy / sqrt(pow(dx,2) + pow(dy,2));
                        long double x_ = E_x_re - (rr * Cos_R);
                        long double y_ = E_y_re - (rr * Sin_R); 
                        long double v = Cam_Return_v(theta0 , re_theta , r0); 
                        long double a = Cam_Return_a(theta0 , re_theta , r0);
                        //long double o1p = 262 / (1 + 432 * (phi_re1 - phi_re)); //压力角
                       //long double o2p = pow(pow(o1p,2) + pow(222,2) - (2 * o1p * cos(phi_re + phi0_re)),0.5 );
                        //long double beta = acos((pow(o2p,2) + pow(222,2) - pow(o1p,2)) / (2 * pow(o2p,2) * 222));
                       //long double alpha = 90 - (beta * 180 / PI);
                        long double alpha = atan((1 / tan(phi_add_phi0_return)) - (222 * (1 - ((phi_re1 - phi_re) / 0.1))) / (262 * sin(phi_add_phi0_return)))* 180 / PI;
                        long double a1= ((222 * (1 - ((phi_re1 - phi_re) / 0.1))) / (262 * sin(phi_add_phi0_return)));
                        fprintf(fp,"%Lf,%Lf,%lf,%Lf,%Lf,%Lf,%Lf,%Lf \n",E_x_re,E_y_re,time_all,x_,y_,alpha,v,a);
                        //printf("p=%lf,\th=%Lf\n",phi_add_phi0_return,half_chord_return);
                        //printf("a = %Lf,\tt = %Lf\t",a1,phi_re1-phi_re);
                        //printf("r1=%Lf\t",return_long);
                        theta_angel ++;
                        time_all ++;
                }
                printf("\n \n");
        
        }

        if (theta_angel >= 348 && theta_angel < 360) {
                int time;

                for (time = 348 ; time < 360 ; ++time ) {
                        double theta = time_all * PI / 180;
                        long double stop_long = r0;
                        long double half_chord = pow((pow(222,2)-pow(((pow(222,2) - pow(stop_long,2) + pow(262,2)) /( 2 * 262)),2)),0.5);
                        long double phi_add_phi0 = asin(half_chord/222);
                        long double phi0 = ConstOF_Phi0(r0);
                        long double phi = phi_add_phi0 - phi0;
                        long double E_x_ = LocationOF_X( theta, phi, phi0);
                        long double E_y_  = LocationOF_Y( theta, phi, phi0);
                        long double Sin_Ex = E_y_ / (pow((pow(E_x_,2) + pow(E_y_,2)),0.5));
                        long double Cos_Ey = E_x_ / (pow((pow(E_x_,2) + pow(E_y_,2)),0.5));
                        long double E_x_stop1 = (stop_long - rr) * Cos_Ey;
                        long double E_y_stop1 = (stop_long - rr) * Sin_Ex;
                        //long double o1p = 262 ;
                        //long double o2p = pow(pow(o1p,2) + pow(222,2) - (2 * o1p * cos(phi + phi0)),0.5 );
                        //long double beta = acos((pow(o2p,2) + pow(222,2) - pow(o1p,2)) / (2 * pow(o2p,2) * 222));
                        // 压力角
                        //long double alpha = 90 - (beta * 180 / PI);
                        long double alpha = atan(1 / tan(phi_add_phi0)) * 180 / PI  ;
                        fprintf(fp,"%Lf,%Lf,%lf,%Lf,%Lf,%Lf,0,0\n",E_x_,E_y_,time_all,E_x_stop1,E_y_stop1,alpha);
                        //printf("%lf",r0);
                        theta_angel ++;
                        time_all ++;
                } 
        }
        fclose(fp);
                

        return 0;

}



