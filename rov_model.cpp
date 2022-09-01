#include "rov_model.h"
#include <cmath>
#include <iostream>



ROV_Model::ROV_Model(QObject *parent) : QObject(parent)
{
    resetModel();
    m = 20;
    cv1[1] = 10.9; cv1[2] = 95.0; cv1[3] = 63.3;
    cv2[1] = 10.9; cv2[2] = 114; cv2[3] = 76;
    cw1[1] = 228.6; cw1[2] = 366; cw1[3] = 366; // kak v rabote Egorova
    cw2[1] = 2.29; cw2[2] = 36.6; cw2[3] = 36.6;
    //Vt[1] = 1; Vt[2] = 1; Vt[3] = 1; Vt[4] = 0; Vt[5] = 0; Vt[6] = 0; // скорость течения
    //Wv[1] = 0; Wv[2] = 0; Wv[3] = 0; Wv[4] = 0; Wv[5] = 0; Wv[6] = 0; //внешние возмущения, лин. скорости([1]-[3], угловые скорости - [4]-[6])
    //h[1]= ; h[2]= ; h[3]= ; // радиус-вектор координат центра водоизмещения
    lambda[1][1] = 50; lambda[2][2] = 101; lambda[3][3] = 101;
    lambda[4][4] = 50; lambda[5][5] = 50; lambda[6][6] = 50;
    Ta[1][1] = 0.6124; Ta[1][2] = 0.6124; Ta[1][3] = -0.6124; Ta[1][4] = -0.6124; Ta[1][5] = 0.6124; Ta[1][6] = 0.6124; Ta[1][7] = -0.6124; Ta[1][8] = -0.6124;
    Ta[2][1] = -0.5; Ta[2][2] = 0.5; Ta[2][3] = 0.5; Ta[2][4] = -0.5; Ta[2][5] = -0.5; Ta[2][6] = 0.5; Ta[2][7] = 0.5; Ta[2][8] = -0.5;
    Ta[3][1] = 0.3536; Ta[3][2] = 0.3536; Ta[3][3] = 0.3536; Ta[3][4] = 0.3536; Ta[3][5] = -0.3536; Ta[3][6] = -0.3536; Ta[3][7] = -0.3536; Ta[3][8] = -0.3536;
    Ta[4][1] = 70.1448; Ta[4][2] = -70.1448; Ta[4][3] = -70.1448; Ta[4][4] = 70.1448; Ta[4][5] = -70.1448; Ta[4][6] = 70.1448; Ta[4][7] = 70.1448; Ta[4][8] = -70.1448;
    Ta[5][1] = 20.2244; Ta[5][2] = 20.2244; Ta[5][3] = -20.2244; Ta[5][4] = -20.2244; Ta[5][5] = -20.2244; Ta[5][6] = -20.2244; Ta[5][7] = 20.2244; Ta[5][8] = 20.2244;
    Ta[6][1] = -92.8926; Ta[6][2] = 92.8926; Ta[6][3] = -92.8926; Ta[6][4] = 92.8926; Ta[6][5] = -92.8926; Ta[6][6] = 92.8926; Ta[6][7] = -92.8926; Ta[6][8] = 92.8926;
    //матрица сил и моментов инерции (проверить вторую матрицу, пока я вбила из мат модели, но кажется там я ошиблась она же не симметрична относительно оси, что странно)
    C[1][1] = 0; C[1][2] = (m+lambda[2][2])*a[20]; C[1][3] = -(m + lambda[3][3])*a[19]; C[1][4] = 0; C[1][5] = 0; C[1][6] = 0;
    C[2][1] = -(m + lambda[1][1])*a[20]; C[2][2] = 0; C[2][3] = (m + lambda[3][3])*a[18]; C[2][4] = 0; C[2][5] = 0; C[2][6] = 0;
    C[3][1] = (m + lambda[1][1])*a[19]; C[3][2] = -(m+lambda[2][2])*a[18]; C[3][3] = 0; C[3][4] = 0; C[3][5] = 0; C[3][6] = 0;
    C[4][1] = 0; C[4][2] = 0; C[4][3] = 0; C[4][4] = 0; C[4][5] = -(J[3]+lambda[6][6])*a[20]; C[4][6] = (J[2]+lambda[5][5])*a[19];
    C[5][1] = 0; C[5][2] = 0; C[5][3] = 0; C[5][4] = (J[3]+lambda[6][6])*a[20]; C[5][5] = 0; C[5][6] = -(J[1]+lambda[4][4])*a[18];
    C[6][1] = 0; C[6][2] = 0; C[6][3] = 0; C[6][4] = -(J[2]+lambda[5][5])*a[19]; C[6][5] = (J[1]+lambda[4][4])*a[18]; C[6][6] = 0;
    J[1] = 4; J[2] = 19.8; J[3] = 19.8; //moment inercii apparata vdol sootvetstvuushih osei
    kd = 3; //koefficient usilenija dvizhitelei
    Td = 0.15; //postojannaya vremeni dvizhitelei
    depth_limit=50;
    max_depth=50;
}

void ROV_Model::model(const float Upnp,const float Upnl,const float Uznp,const float Uznl, const float Upvp, const float Upvl, const float Uzvl, const float Uzvp) {
    int limit1, limit2;
    double G;

    //модули упоров движителей
    Ppnp = a[7];  // передний нижний правый(1)
    Ppnl = a[8];  // передний нижний левый(2)
    Pznp = a[9];  // задний нижний левый(3)
    Pznl = a[10];  //задний нижний правый(4)
    Ppvp = a[11];  // передний верхний правый(5)
    Ppvl = a[12];  // передний верхний левый(6)
    Pzvl = a[13];  // задний верхний левый(7)
    Pzvp = a[14];  //задний верхний правый(8)

    //проекции упоров движителей на продольную ось апарата X
    Ppnp_x = Ppnp*Ta[1][1];
    Ppnl_x = Ppnl*Ta[1][2];
    Pznp_x = Pznp*Ta[1][3];
    Pznl_x = Pznl*Ta[1][4];
    Ppvp_x = Ppvp*Ta[1][5];
    Ppvl_x = Ppvl*Ta[1][6];
    Pzvl_x = Pzvl*Ta[1][7];
    Pzvp_x = Pzvp*Ta[1][8];

    //проекции упоров движителей на продольную ось апарата Y
    Ppnp_y = Ppnp*Ta[2][1];
    Ppnl_y = Ppnl*Ta[2][2];
    Pznp_y = Pznp*Ta[2][3];
    Pznl_y = Pznl*Ta[2][4];
    Ppvp_y = Ppvp*Ta[2][5];
    Ppvl_y = Ppvl*Ta[2][6];
    Pzvl_y = Pzvl*Ta[2][7];
    Pzvp_y = Pzvp*Ta[2][8];

    //проекции упоров движителей на продольную ось апарата Z
    Ppnp_z = Ppnp*Ta[3][1];
    Ppnl_z = Ppnl*Ta[3][2];
    Pznp_z = Pznp*Ta[3][3];
    Pznl_z = Pznl*Ta[3][4];
    Ppvp_z = Ppvp*Ta[3][5];
    Ppvl_z = Ppvl*Ta[3][6];
    Pzvl_z = Pzvl*Ta[3][7];
    Pzvp_z = Pzvp*Ta[3][8];

    //момент создаваемый движетельным комплексом вокруг оси X
    Mpnp_x = Ppnp*Ta[4][1];
    Mpnl_x = Ppnl*Ta[4][2];
    Mznp_x = Pznp*Ta[4][3];
    Mznl_x = Pznl*Ta[4][4];
    Mpvp_x = Ppvp*Ta[4][5];
    Mpvl_x = Ppvl*Ta[4][6];
    Mzvl_x = Pzvl*Ta[4][7];
    Mzvp_x = Pzvp*Ta[4][8];

    //момент создаваемый движетельным комплексом вокруг оси Y
    Mpnp_y = Ppnp*Ta[5][1];
    Mpnl_y = Ppnl*Ta[5][2];
    Mznp_y = Pznp*Ta[5][3];
    Mznl_y = Pznl*Ta[5][4];
    Mpvp_y = Ppvp*Ta[5][5];
    Mpvl_y = Ppvl*Ta[5][6];
    Mzvl_y = Pzvl*Ta[5][7];
    Mzvp_y = Pzvp*Ta[5][8];

    //момент создаваемый движетельным комплексом вокруг оси Z
    Mpnp_z = Ppnp*Ta[6][1];
    Mpnl_z = Ppnl*Ta[6][2];
    Mznp_z = Pznp*Ta[6][3];
    Mznl_z = Pznl*Ta[6][4];
    Mpvp_z = Ppvp*Ta[6][5];
    Mpvl_z = Ppvl*Ta[6][6];
    Mzvl_z = Pzvl*Ta[6][7];
    Mzvp_z = Pzvp*Ta[6][8];

    double g = 9.81;
    G = m*g; //вес аппарата
    Fa = 200;
    Farx[0] = 0; Farx[1] = 0; Farx[2] = -Fa;

    //obnulenie verticalnoi polozhitelnoi skorosti apparata pri dostizhenii poverhnosti
    limit1 = limit2 = 0;
    if (a[17] >= max_depth) {
      a[17] = max_depth;
        if (a[3] <= 0) {
          a[3] = 0;
          limit1 = 1;
      }
    };

    //obnulenie verticalnoi polozhitelnoi skorosti apparata pri dostizhenii dna
    if (a[17] <= 0)
    {
      a[17] = 0;
        if (a[3] >= 0)
      {
          a[3] = 0;
          limit2 = 1;
      }
    };

    Fdx = Ppnp_x + Ppnl_x + Pznp_x + Pznl_x + Ppvp_x + Ppvl_x + Pzvl_x + Pzvp_x; // вектор сил и моментов, создаваемых движительным комплексом
    Fgx = -cv1[1] * a[1] * fabs(a[1]) - cv2[1] * a[1]; //произведение D1*Vx
    FloatageX = -sin(a[5]) * (G + Farx[2]);
    Fcx = C[1][1]*a[1] + C[1][2]*a[2]+C[1][3]*a[3]+C[1][4]*a[18]+C[1][5]*a[19] + C[1][6]*a[20];
    //FloatageX = 0; //обнуление плавучести
    da[1] = (1/(m + lambda[1][1])) * (Fdx + Fgx + Fcx + FloatageX + Wv[1]); //vx'

    Fdy = Ppnp_y + Ppnl_y + Pznp_y + Pznl_y + Ppvp_y + Ppvl_y + Pzvl_y + Pzvp_y; // вектор сил и моментов, создаваемых движительным комплексом
    Fgy = -cv1[2] * a[2] * fabs(a[2]) - cv2[2] * a[2]; //произведение D1*Vy
    FloatageY = cos(a[5]) * sin(a[4]) * (G + Farx[2]);
    Fcy = C[2][1]*a[1] + C[2][2]*a[2]+C[2][3]*a[3]+C[2][4]*a[18]+C[2][5]*a[19] + C[2][6]*a[20];
    //FloatageY = 0; //обнуление плавучести
    da[2] = (1/(m + lambda[2][2])) * (Fdy + Fgy + Fcy + FloatageY + Wv[2]); //vy'

    Fdz = Ppnp_z + Ppnl_z + Pznp_z + Pznl_z + Ppvp_z + Ppvl_z + Pzvl_z + Pzvp_z; // вектор сил и моментов, создаваемых движительным комплексом
    Fgz = -cv1[3] * a[3] * fabs(a[3]) - cv2[3] * a[3]; //произведение D1*Vz
    FloatageZ = cos(a[4]) * cos(a[5]) * (G + Farx[2]);
    Fcz = C[3][1]*a[1] + C[3][2]*a[2]+C[3][3]*a[3]+C[3][4]*a[18]+C[3][5]*a[19] + C[3][6]*a[20];
    //FloatageZ = 0; //обнуление плавучести
    da[3] = (1/(m + lambda[3][3])) * (Fdz + Fgz + Fcz + FloatageZ + Wv[3]); //vz'

// da[4-6] -> производная угла крена, дифферента, курса
//следующие 3 уравнения это Кинематические уравнения для углов Эйлера-Крылова
//описывающее преобразование вектора угловых скоростей относительно осей НПА Ox,Oy,Oz в вектор
//угловых скоростей  по курсу, дифференту и крену соответственно.

    da[4] = a[18] + (1/cos(a[5]) * ((a[19]) * sin(a[4]) * sin(a[5])  + sin(a[5]) * cos(a[4]) * a[20])) + Vt[4];  //proizvodnaya krena

    da[5] = a[19] * cos(a[4]) - sin(a[4]) * a[20] + Vt[5];  //proizvodnaya differenta

    da[6] = (1/cos(a[5])) * (a[19] * sin(a[4]) + cos(a[4]) * (a[20])) + Vt[6]; //proizvodnaya kursa
 // Из матмодели имеем
 //K_двi - усредненный коэффициент усиления i-го движителя; T_двi=J_i/K_v1i  – наибольшее значение постоянной времени i-го ВМА
    da[7] = (1/Td) * (kd * (double)Upnp - Ppnp);  // передний нижний правый(1)
    da[8] = (1/Td) * (kd * (double)Upnl - Ppnl);  // передний нижний левый(2)
    da[9] = (1/Td) * (kd * (double)Uznp - Pznp);  // задний нижний левый(3)
    da[10] = (1/Td) * (kd * (double)Uznl - Pznl); //задний нижний правый(4)
    da[11] = (1/Td) * (kd * (double)Upvp - Ppvp); // передний верхний правый(5)
    da[12] = (1/Td) * (kd * (double)Upvl - Ppvl); // передний верхний левый(6)
    da[13] = (1/Td) * (kd * (double)Uzvl - Pzvl); //задний верхний правый(8)
    da[14] = (1/Td) * (kd * (double)Uzvp - Pzvp); // задний верхний левый(7)


    double alfa[4][4]; //матрица перевода из связанной СК в глобальную СК
    alfa[1][1] = cos(a[5])*cos(a[6]);
    alfa[2][1] = sin(a[6])*cos(a[5]);
    alfa[3][1] = -sin(a[5]);
    alfa[1][2] = cos(a[6])*sin(a[5])*sin(a[4])-cos(a[4])*sin(a[6]);
    alfa[2][2] = cos(a[6])*cos(a[4])+sin(a[4])*sin(a[5])*sin(a[6]);
    alfa[3][2] = sin(a[4])*cos(a[5]);
    alfa[1][3] = sin(a[6])*sin(a[4])+cos(a[6])*cos(a[4])*sin(a[5]);
    alfa[2][3] = sin(a[5])*sin(a[6])*cos(a[4])-cos(a[6])*sin(a[4]);
    alfa[3][3] = cos(a[5])*cos(a[4]);

    da[15] = alfa[1][1] * a[1] + alfa[1][2] * a[2] + alfa[1][3] * a[3] + Vt[1];
    //dx_global

    da[16] = alfa[2][1] * a[1] + alfa[2][2] * a[2] + alfa[2][3] * a[3] + Vt[2];
    //dy_global

    da[17] = alfa[3][1] * a[1] + alfa[3][2] * a[2] + alfa[3][3] * a[3] + Vt[3];
    //dz_global

    double Fax = -sin(a[5])*Fa;
    double Fay = sin(a[4])*cos(a[5])*Fa;
    double Faz = cos(a[5])*cos(a[4])*Fa;

    Mdx = Mpnp_x + Mpnl_x + Mznp_x + Mznl_x + Mpvp_x + Mpvl_x + Mzvl_x + Mzvp_x;
    Mgx = -cw1[1] * a[18] * fabs(a[18]) - cw2[1] * a[18];
    Max = -h[2]*Faz + h[3]*Fay;
    //Max = 0; //obnulenie momenta ot sily Arhimeda
    Mcx = C[4][1]*a[1] + C[4][2]*a[2]+C[4][3]*a[3]+C[4][4]*a[18]+C[4][5]*a[19] + C[4][6]*a[20];
    da[18] = (1/(J[1] + lambda[4][4])) * (Mdx + Mcx + Mgx + Max + Wv[4]);

    Mdy = Mpnp_y + Mpnl_y + Mznp_y + Mznl_y + Mpvp_y + Mpvl_y + Mzvl_y + Mzvp_y;
    Mgy = -cw1[2] * a[19] * fabs(a[19]) - cw2[2] * a[19];
    May = -Faz*h[1] + Fax*h[3];
    //May = 0; //obnulenie momenta ot sily Arhimeda
    Mcy = C[5][1]*a[1] + C[5][2]*a[2]+C[5][3]*a[3]+C[5][4]*a[18]+C[5][5]*a[19] + C[5][6]*a[20];
    da[19] = (1/(J[2] + lambda[5][5])) * (Mdy + Mcy + Mgy + May + Wv[5]);

    Mdz = Mpnp_z + Mpnl_z + Mznp_z + Mznl_z + Mpvp_z + Mpvl_z + Mzvl_z + Mzvp_z;
    Mgz = -cw1[3] * a[20] * fabs(a[20]) - cw2[3] * a[20];
    Maz = -h[1]*Fay + h[2]*Fax;
    //Maz = 0; //obnulenie momenta ot sily Arhimeda
    Mcz = C[6][1]*a[1] + C[6][2]*a[2]+C[6][3]*a[3]+C[6][4]*a[18]+C[6][5]*a[19] + C[6][6]*a[20];
    da[20] = (1/(J[3] + lambda[6][6])) * (Mdz + Mcz + Mgz + Maz + Wv[6]);

    da[21] = a[1];
    da[22] = a[2];
    da[23] = a[3];

}

void ROV_Model::resetModel(){
    for (int i=0;i<ANPA_MOD_CNT;i++) {a[i] = 0.0f; da[i]=0.0f;}   //f на конце означает число с плавающей точкой
    for (int i=0; i<7;i++){
        Wv[i]=0;
        Vt[i]=0;
        h[i]=0;   //потом исправить на реальное значение сверху
    }
}

void ROV_Model::tick(const float Upnp,const float Upnl,const float Uznp,const float Uznl,
                     const float Upvp, const float Upvl, const float Uzvl, const float Uzvp,const float Ttimer){

    runge(Upnp, Upnl, Uznp, Uznl, Upvp, Upvl, Uzvl, Uzvp,Ttimer,Ttimer);
}

ROV_Model::~ROV_Model(){

}

void ROV_Model::runge(const float Upnp,const float Upnl,const float Uznp,const float Uznl,
                      const float Upvp, const float Upvl, const float Uzvl, const float Uzvp, const float Ttimer, const float dt) {
    const double Kc = 180/M_PI;
    double a1[24], y[24];
    int i;
    const double H1 = dt;
    const int n = ANPA_MOD_CNT;
    model(Upnp, Upnl, Uznp, Uznl, Upvp, Upvl, Uzvl, Uzvp);
    for (i = 1; i < n; i++) {
      a1[i] = a[i];
      y[i] = da[i];
      a[i] = a1[i] + 0.5 * H1 * da[i];
    }

    model(Upnp, Upnl, Uznp, Uznl, Upvp, Upvl, Uzvl, Uzvp);
    for (i = 1; i < n; i++)
    {
      y[i] = y[i]+ 2 * da[i];
      a[i] = a1[i] + 0.5 * H1 * da[i];
    }

    model(Upnp, Upnl, Uznp, Uznl, Upvp, Upvl, Uzvl, Uzvp);
    for (i = 1; i < n; i++) {
      y[i] = y[i] + 2 * da[i];
      a[i] = a1[i] + H1 * da[i];
    }

    model(Upnp, Upnl, Uznp, Uznl, Upvp, Upvl, Uzvl, Uzvp);
    for (i = 1; i < n; i++) {
      a[i] = a1[i] + (H1 / 6) * (y[i] + da[i]);
    }

    //данные в СУ ( с преобразованием координат)

    x_global = a[15]; //koordinata apparata v globalnoi SK
    y_global = a[16];  //koordinaty apparata v globalnoi SK (преобразование координат)
    z_global = a[17]; //otstojanie ot dna otnositelno repernoi tochki, kotoraja na dne
    cur_depth = max_depth + z_global;  //tekush"aya glubina SPA
    Wx = a[18] * Kc; //uglovye skorosti SPA v svyazannyh osyah v gradus/sekunda
    Wy = a[19] * Kc;
    Wz = a[20] * Kc;

    vx_local = a[1]; vy_local = a[2]; vz_local = a[3];  //lineinye skorosti SPA v svyazannyh osyah
    vx_global = da[15]; vy_global = da[16]; vz_global = da[17];  // lineinye skorosti SPA v globalnyh osyah

    Gamma_g = a[4] * Kc; // ugol krena
    Tetta_g = a[5] * Kc; // ugol differenta
    Psi_g = a[6] * Kc; // ugol kursa (преобразование координат)

    W_Gamma_g = da[4] * Kc; // proizvodnaya ugla krena
    W_Tetta_g = da[5] * Kc; // proizvodnaya ugla differenta
    W_Psi_g = da[6] * Kc; // proizvodnaya ugla kursa

    N = fabs(Psi_g / 360);
    if (Psi_g >= 360) Psi_gi = Psi_g - N * 360; // ugol kursa na indikaciu
    if (Psi_g <= -360) Psi_gi = Psi_g + N * 360;

    deltaSx = vx_local * Ttimer; //prirash"enie koordinaty X dlya SVS (v svyazannoi s SPA SK)
    sumX += deltaSx;

    deltaSz = vz_local * Ttimer; //prirash"enie koordinaty Z dlya SVS (v svyazannoi s SPA SK)
    sumZ += deltaSz;

    X[5][0] = Gamma_g;
    X[6][0] = Tetta_g;
    X[7][0] = Psi_g;

    X[10][0]=Wx;
    X[11][0]=Wy;
    X[12][0]=Wz;

    X[13][0]=vx_local;
    X[14][0]=vy_local;
    X[15][0]=vz_local;

    X[16][0]=W_Gamma_g;
    X[17][0]=W_Tetta_g;
    X[18][0]=W_Psi_g;

    X[19][0]=x_global;
    X[20][0]=y_global;
    X[21][0]=z_global;

    X[22][0]=Ppnp;
    X[23][0]=Ppnl;
    X[24][0]=Pznp;
    X[25][0]=Pznl;
    X[26][0]=Ppvp;
    X[27][0]=Ppvl;
    X[28][0]=Pzvl;
    X[29][0]=Pzvp;
}

