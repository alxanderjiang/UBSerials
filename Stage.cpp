#include "Stage.h"

#pragma 结构体和宏定义

#define SECOND 315964800
//定义Unix时结构
struct gtime_t {
    time_t time;
    double second;
};
//定义GPS时结构
struct GPSTime {
    unsigned short Week;
    double Second;
};
//定义通用计时结构
struct COMMONTIME {
    unsigned short Year;
    unsigned short Month;
    unsigned short Day;
    unsigned short Hour;
    unsigned short Minute;
    double Second;
};

#define ea 6378137.0
#define pi 3.1415926535897932384626433832795
#define es2 0.00669437999013//第一偏心率的平方
#define e22 0.006739496742227//第二偏心率的平方
//#define es2 0.0818191910428*0.0818191910428//第一偏心率的平方
//#define e22 0.0820944381519*0.0820944381519//第二偏心率的平方
#define clight 299792458.0
#pragma 子窗口单独引用运算函数重构

struct gtime_t com2unixtime_t(struct COMMONTIME t0) {
    struct gtime_t t;
    int i = 0;
    //开始计算通用时距Unixtime起点时的Days
    int Days = 0;
    int doy[] = { 1,32,60,91,121,152,182,213,244,274,305,335 };
    Days = (t0.Year - 1970) * 365 + (t0.Year - 1969) / 4 + doy[t0.Month - 1] + t0.Day - 2 + (t0.Year % 4 == 0 && t0.Month >= 3 ? 1 : 0);//老师提供算法，对2000年以前时间表现出暂时有误性
    t.time = Days * 86400 + t0.Hour * 3600 + t0.Minute * 60 + floor(t0.Second);
    t.second = t0.Second - floor(t0.Second);
    return  t;
}
//GPS时转Unix time
struct gtime_t gpst2time_t(int week, double seconds) {
    gtime_t result = {};
    result.second = seconds - floor(seconds);
    result.time = time_t(86400 * 7 * week) + int(seconds) + SECOND;
    return result;
}

double time2gpst_t(gtime_t t0, int& week) {
    time_t sec = t0.time - SECOND;
    week = int(sec / (86400 * 7));
    double second = (double)(sec - week * 86400 * 7) + t0.second;
    return second;
}
//unix时转通用时
struct COMMONTIME time2epoch_t(gtime_t t0) {
    int Days = (int)(t0.time) / 86400;
    double secs = (int)(t0.time - (time_t)Days * 86400);

    int mday[] = { 31,28,31,30,31,30,31,31,30,31,30,31,
                31,28,31,30,31,30,31,31,30,31,30,31,
                31,29,31,30,31,30,31,31,30,31,30,31,
                31,28,31,30,31,30,31,31,30,31,30,31 };
    int d, mon;
    for (d = Days % 1461, mon = 0; mon < 48; mon++) {
        if (d >= mday[mon])
            d -= mday[mon];
        else
            break;
    }

    struct COMMONTIME comtime;
    comtime.Year = 1970 + Days / 1461 * 4 + mon / 12;
    comtime.Month = mon % 12 + 1;
    comtime.Day = d + 1;
    comtime.Hour = secs / 3600;
    comtime.Minute = (secs - comtime.Hour * 3600) / 60;
    comtime.Second = secs - comtime.Hour * 3600 - comtime.Minute * 60;
    return comtime;
}
//直角转大地
void xyztoblh_t(double x, double y, double z, double* str) {
    double tb, h, b, l, n;
    double temp1, temp2;
    double m1, m2, m3, m4;
    l = acos(x / sqrt(pow(x, 2) + pow(y, 2)));
    m1 = 1 / sqrt(pow(x, 2) + pow(y, 2));
    m2 = ea * es2;
    m3 = 1 - es2;
    temp1 = z / sqrt(pow(x, 2) + pow(y, 2));
    //以下为迭代
    do {
        temp2 = temp1;
        temp1 = m1 * (z + m2 * temp1 / sqrt(1 + m3 * pow(temp1, 2)));
    } while (temp1 - temp2 > 1e-14);
    tb = temp1; b = atan(tb);
    n = ea / sqrt(1 - es2 * pow(sin(b), 2));
    h = sqrt(pow(x, 2) + pow(y, 2)) / cos(b) - ea / sqrt(1 - es2 * pow(sin(b), 2));
    if (y < 0) l = -l;
    str[0] = b * 180 / pi, str[1] = l * 180 / pi, str[2] = h;//储存结果
    //printf("大地坐标(%.4lf,%.4lf,%.4lf)\n",str[0],str[1],str[2]);
}
//大地转直角
void blhtoxyz_t(double b, double l, double h, double* str) {
    double x, y, z;
    double n;
    b = b * pi / 180; l = l * pi / 180;
    n = ea / sqrt(1 - es2 * pow(sin(b), 2));
    x = (n + h) * cos(b) * cos(l);
    y = (n + h) * cos(b) * sin(l);
    z = (n * (1 - es2) + h) * sin(b);
    str[0] = x, str[1] = y, str[2] = z;//储存结果
    //printf("空间直角坐标(%.10lf,%.10lf,%.10lf)\n",x,y,z);
}
//矩阵乘
void matx_t(double* A, int rowa, int cola, double* B, int rowb, int colb, double* C) {
    if (cola != rowb) { printf("矩阵不可乘"); return; }
    int i, j, k, t;
    for (i = 0; i < rowa; i++)
        for (j = 0; j < colb; j++) {
            for (k = i * cola, t = j; k < i * cola + cola; k++, t += colb)
                C[i * colb + j] += A[k] * B[t];
        }
}
//以下函数功能为计算卫星方向角和高度角
void getazel_t(const double* rs, const double* rr, double* azel) {
    //从参数接受卫星和接收机坐标同时转化为大地坐标
    //double ion[3]={},pos[3]={};
    double rsblh[3]; xyztoblh_t(rs[0], rs[1], rs[2], rsblh);
    double rrblh[3]; xyztoblh_t(rr[0], rr[1], rr[2], rrblh);//错误点
    //请注意xyztoblh函数返回值的格式是角度制还是弧度制，本代码为角度制
    double d = sqrt(pow(rs[0] - rr[0], 2) + pow(rs[1] - rr[1], 2) + pow(rs[2] - rr[2], 2));
    double e1, e2, e3, e[3];

    e1 = (rs[0] - rr[0]) / d; e2 = (rs[1] - rr[1]) / d; e3 = (rs[2] - rr[2]) / d;
    e[0] = rs[0] - rr[0]; e[1] = rs[1] - rr[1]; e[2] = rs[2] - rr[2];
    double B = rrblh[0] * pi / 180, L = rrblh[1] * pi / 180;//测站的大地坐标，转换为弧度制以计算三角函数

    //地心坐标系转站心坐标系
    double E, N, U, ER[3] = {};
    double H[9] = { -sin(L),cos(L),0,
                -sin(B) * cos(L),-sin(B) * sin(L),cos(B),
                cos(B) * cos(L),cos(B) * sin(L),sin(B) };

    matx_t(H, 3, 3, e, 3, 1, ER);
    E = ER[0]; N = ER[1]; U = ER[2];

    //卫星高度角/方向角的计算
    double az, el;
    az = atan2(E, N);
    el = asin(U / d);
    azel[0] = az;
    azel[1] = el;
}
//以下函数功能为计算电离层延迟
double ionmodel_t(GPSTime t, const double* ion, const double* pos, const double* azel) {
    //从参数中取出需要的数值
    double az = azel[0];
    double el = azel[1];
    double rrblh[3]; xyztoblh_t(pos[0], pos[1], pos[2], rrblh);
    //计算地球中心角
    double Phi = 0.0137 / ((el / pi) + 0.11) - 0.022;
    //计算电离层穿刺点的纬度phi1
    double phi1, phiu;
    phiu = rrblh[0] / 180.0;
    phi1 = phiu + Phi * cos(az);
    if (phi1 > 0.416) phi1 = 0.416;
    else if (phi1 < -0.416) phi1 = -0.416;
    //计算电离层穿刺点经度lamda1
    double lamda1, lamdau;
    lamdau = rrblh[1] / 180.0;
    lamda1 = lamdau + Phi * sin(az) / cos(phi1 * pi);
    //计算电离层穿刺点的地磁纬度phim
    double phim;
    phim = phi1 + 0.064 * cos((lamda1 - 1.617) * pi);
    //计算电离层穿刺点的当地时间localtime
    double localtime;
    localtime = 43200 * lamda1 + t.Second;//计算当地时（以GPS周内秒为基准）
    localtime = localtime - floor(localtime / 86400.0) * 86400;//扣除整数天数，得到一天内的地方时秒数
    //计算电离层延迟的幅度A1
    double A1;
    A1 = ion[0] + phim * (ion[1] + phim * (ion[2] + phim * ion[3]));
    if (A1 < 0) A1 = 0;
    //计算电离层延迟的周期P1
    double P1;
    P1 = ion[4] + phim * (ion[5] + phim * (ion[6] + phim * ion[7]));
    if (P1 < 72000) P1 = 72000;
    //计算电离层延迟相位X1
    double X1;
    X1 = 2 * pi * (localtime - 50400) / P1;
    //计算倾斜因子F
    double F;
    F = 1.0 + 16.0 * pow((0.53 - el / pi), 3);

    //模型参数计算完毕，下面根据模型计算电离层延迟IL1GPS
    double IL1GPS;
    if (fabs(X1) <= 1.57)
        IL1GPS = clight * (5 * (1e-9) + A1 * (1 - 0.5 * X1 * X1 + pow(X1, 4) / 24.0)) * F;
    else
        IL1GPS = 5 * (1e-9) * clight * F;
    double IGS1 = clight * (5 * (1e-9) + A1 * (1 - 0.5 * X1 * X1 + pow(X1, 4) / 24.0)) * F;
    double IGS2 = 5 * (1e-9) * clight * F;
    return IL1GPS;
}

double ionmodel_BDS_t(GPSTime tobs, const double* ion, const double* pos, const double* azel) {
    //从参数中读取卫星高度角方位角
    double az = azel[0], el = azel[1];
    //从参数中读取地理坐标
    double rblh[3] = {};
    xyztoblh_t(pos[0], pos[1], pos[2], rblh);
    //定义地球半径和电离层高度
    double Re = 6378000.0, hion = 375000.0;
    //用户和穿刺点的地心张角
    double angel = pi / 2 - el - asin(Re / (Re + hion) * cos(el));
    //电离层穿刺点的地理纬度、地理经度
    double phiM = asin(sin(rblh[0]) * cos(angel) + cos(rblh[0]) * sin(angel) * cos(az));
    double lambdaM = rblh[1] + asin(sin(angel) * sin(az) / cos(phiM));
    //余弦曲线周期
    double A4, x = phiM / pi;
    A4 = ion[4] + ion[5] * x + ion[6] * x * x + ion[7] * x * x * x;
    A4 = (A4 >= 172800 ? 172800 : A4);
    A4 = (A4 < 72000 ? 72000 : A4);
    //白天电离层延迟余弦曲线的幅度
    double A2;
    A2 = ion[0] + ion[1] * x + ion[2] * x * x + ion[3] * x * x * x;
    A2 = (A2 < 0 ? 0 : A2);
    //穿刺点地方时
    double t = (tobs.Second - 14 + lambdaM * 43200.0 / pi);
    //垂直延迟改正
    double Iz;
    if (abs(t - 50400) >= A4 / 4)
        Iz = 5e-9;
    else
        Iz = 5e-9 + A2 * cos(2 * pi * (t - 50400) / A4);
    //B1信号延迟
    double IzB1I = 1.0 / sqrt(1 - (Re / (Re + hion) * cos(el)) * (Re / (Re + hion) * cos(el))) * Iz;
    //秒归距
    return IzB1I * clight;
}
//以下函数功能为对流层延迟计算
double tropmodel_t(const double* pos, const double* azel) {
    //将直角坐标转换为大地坐标
    double posblh[3]; xyztoblh_t(pos[0], pos[1], pos[2], posblh);
    //不在地球上，对流层延迟归零
    if (posblh[2] < -100.0 || 1E4 < posblh[2] || azel[1] <= 0) return 0.0;

    double humi = 0.7;
    double h = posblh[2], b = posblh[0] * pi / 180.0;//因为在头文件中坐标转换函数定义为角度值，所以计算前需要复原
    if (posblh[2] < 0.0) h = 0.0;//地面高程归零处理

    double T = 15.0 - 6.5 * 1e-3 * h + 273.16;
    double e = 6.108 * humi * exp((17.15 * T - 4684.0) / (T - 38.45));
    double p = 1013.25 * pow((1 - 2.2557e-5 * h), 5.2568);
    double z = pi / 2.0 - azel[1];
    double trph = 0.0022768 * p / (cos(z) * (1.0 - 0.00266 * cos(2 * b) - 0.00028 * h / 1000.0));
    double trpw = 0.002277 * (1255.0 / T + 0.05) * e / cos(z);
    double trp = trph + trpw;
    return trp;
}
//以下函数功能为字符串特定字符替换(c->b)
void charreplace_t(char* str, char c, char b) {
    for (int i = 0; i < strlen(str); i++)
        if (str[i] == c)
            str[i] = b;
}

#pragma 时间转换区
void Stage::TimetransInit(void) {
    

    QLabel* GPSTlabel = new QLabel("GPST:", this);
    QLabel* GPSTweeklabel = new QLabel("weeks", this);
    QLabel* GPSTsecondslabel = new QLabel("seconds", this);

    QLabel* Unixtimelabel = new QLabel("UnixT:", this);
    QLabel* Unixtimelabel1 = new QLabel("times", this);
    QLabel* Unixtimelabel2 = new QLabel("seconds", this);

    GPSTweek = new QPlainTextEdit(this);
    GPSTseconds = new QPlainTextEdit(this);

    UnixTtimes = new QPlainTextEdit(this);
    UnixTseconds = new QPlainTextEdit(this);

    Yearedit = new QPlainTextEdit(this);
    Monthedit = new QPlainTextEdit(this);
    Dayedit = new QPlainTextEdit(this);
    Houredit = new QPlainTextEdit(this);
    Minuteedit = new QPlainTextEdit(this);
    Secondsedit = new QPlainTextEdit(this);

    GPSTlabel->move(50, 30);
    GPSTlabel->setFixedSize(100, 50);
    GPSTweek->setFixedSize(200, 50);
    GPSTweek->move(150, 30);
    GPSTweeklabel->move(350, 30);
    GPSTweeklabel->setFixedSize(100, 50);
    GPSTseconds->setFixedSize(200, 50);
    GPSTseconds->move(600, 30);
    GPSTsecondslabel->move(800, 30);
    GPSTsecondslabel->setFixedSize(100, 50);

    Unixtimelabel->setFixedSize(100, 50);
    Unixtimelabel->move(50, 120);
    UnixTtimes->setFixedSize(200, 50);
    UnixTtimes->move(150, 120);
    Unixtimelabel1->move(350, 120);
    Unixtimelabel1->setFixedSize(100, 50);
    UnixTseconds->setFixedSize(200, 50);
    UnixTseconds->move(600, 120);
    Unixtimelabel2->move(800, 120);
    Unixtimelabel2->setFixedSize(100, 50);

    Yearedit->setFixedSize(100, 50);
    Yearedit->move(50, 200);
    QLabel* text1 = new QLabel("年", this);
    text1->move(150, 210);
    Monthedit->setFixedSize(100, 50);
    Monthedit->move(200, 200);
    QLabel* text2 = new QLabel("月", this);
    text2->move(300, 210);
    Dayedit->setFixedSize(100, 50);
    Dayedit->move(350, 200);
    QLabel* text3 = new QLabel("日", this);
    text3->move(450, 210);
    Houredit->setFixedSize(100, 50);
    Houredit->move(500, 200);
    QLabel* text4 = new QLabel("时", this);
    text4->move(600, 210);
    Minuteedit->setFixedSize(100, 50);
    Minuteedit->move(650, 200);
    QLabel* text5 = new QLabel("分", this);
    text5->move(750, 210);
    Secondsedit->setFixedSize(200, 50);
    Secondsedit->move(800, 200);
    QLabel* text6 = new QLabel("秒", this);
    text6->move(1000, 210);
    
    
    Timetransbt = new QPushButton("时间转换", this);
    Timetransbt->setFixedSize(200, 50);
    Timetransbt->move(950, 30);
    
    
    connect(Timetransbt, &QPushButton::clicked, [&]() {
        QVector<QPlainTextEdit*>temp;
        temp.push_back(GPSTweek);
        temp.push_back(GPSTseconds);
        temp.push_back(UnixTtimes);
        temp.push_back(UnixTseconds);
        temp.push_back(Yearedit);
        temp.push_back(Monthedit);
        temp.push_back(Dayedit);
        temp.push_back(Houredit);
        temp.push_back(Minuteedit);
        temp.push_back(Secondsedit);
        QVector<QString>tempstr;
        tempstr.setSharable(true);
        for (int i = 0; i < temp.size(); i++) {
            tempstr.push_back(temp[i]->toPlainText());
        }
        
        if (!tempstr[0].isEmpty() && !tempstr[1].isEmpty() &&
            tempstr[2].isEmpty()&& tempstr[3].isEmpty() && 
            tempstr[4].isEmpty()&& tempstr[5].isEmpty() && 
            tempstr[6].isEmpty()&& tempstr[7].isEmpty() && 
            tempstr[8].isEmpty()&& tempstr[9].isEmpty()) 
        {
            gtime_t ut = gpst2time_t(tempstr[0].toInt(), tempstr[1].toDouble());
            Refreshtimeshow(ut.time, ut.second);
        }
        else if (tempstr[0].isEmpty() && tempstr[1].isEmpty() &&
            !tempstr[2].isEmpty() && !tempstr[3].isEmpty() &&
            tempstr[4].isEmpty() && tempstr[5].isEmpty() &&
            tempstr[6].isEmpty() && tempstr[7].isEmpty() &&
            tempstr[8].isEmpty() && tempstr[9].isEmpty()) 
        {
            Refreshtimeshow(tempstr[2].toLongLong(), tempstr[3].toDouble());
        }
        else if (tempstr[0].isEmpty() && tempstr[1].isEmpty() && tempstr[2].isEmpty()&&
            tempstr[3].isEmpty() && !tempstr[4].isEmpty() &&
            !tempstr[5].isEmpty() && !tempstr[6].isEmpty() &&
            !tempstr[7].isEmpty() && !tempstr[8].isEmpty() &&
            !tempstr[9].isEmpty()) 
        {
            COMMONTIME ct = { tempstr[4].toInt(),tempstr[5].toInt(), tempstr[6].toInt(),
            tempstr[7].toInt(), tempstr[8].toInt(), tempstr[9].toDouble() };
            gtime_t ut = com2unixtime_t(ct);
            Refreshtimeshow(ut.time, ut.second);
        }
        else
            QMessageBox::critical(this,"时间输入格式错误", "请输入正确的待转换时间");
        });

    Timetransclearbt = new QPushButton("清空时间转换区", this);
    Timetransclearbt->setFixedSize(200, 50);
    Timetransclearbt->move(950, 100);
    connect(Timetransclearbt, &QPushButton::clicked, [&]() {
        QVector<QPlainTextEdit*>temp;
        temp.push_back(GPSTweek);
        temp.push_back(GPSTseconds);
        temp.push_back(UnixTtimes);
        temp.push_back(UnixTseconds);
        temp.push_back(Yearedit);
        temp.push_back(Monthedit);
        temp.push_back(Dayedit);
        temp.push_back(Houredit);
        temp.push_back(Minuteedit);
        temp.push_back(Secondsedit);
        
        for (int i = 0; i < temp.size(); i++) {
            temp[i]->clear();
        }
        });

}
void Stage::Refreshtimeshow(time_t time, double seconds) {
    gtime_t t = { time,seconds };
    GPSTime gpst = {}; COMMONTIME ct = {};
    int week = 0; gpst.Second = time2gpst_t(t, week); gpst.Week = week;
    ct = time2epoch_t(t);
    QString plaintext;
    GPSTweek->setPlainText(plaintext.sprintf("%d", gpst.Week));
    GPSTseconds->setPlainText(plaintext.sprintf("%lf", gpst.Second));
    UnixTtimes->setPlainText(plaintext.sprintf("%lld", t.time));
    UnixTseconds->setPlainText(plaintext.sprintf("%lf", t.second));
    Yearedit->setPlainText(plaintext.sprintf("%d", ct.Year));
    Monthedit->setPlainText(plaintext.sprintf("%d", ct.Month));
    Dayedit->setPlainText(plaintext.sprintf("%d", ct.Day));
    Houredit->setPlainText(plaintext.sprintf("%d", ct.Hour));
    Minuteedit->setPlainText(plaintext.sprintf("%d", ct.Minute));
    Secondsedit->setPlainText(plaintext.sprintf("%lf", ct.Second));
}

#pragma 坐标转换区
void Stage::CdTransInit(void) {
    XYZedit = new QPlainTextEdit(this);
    XYZedit->setFixedSize(800, 50);
    XYZedit->move(50, 350);
    QLabel* XYZlabel = new QLabel("直角坐标(X Y Z)", this);
    XYZlabel->setFixedSize(200, 50);
    XYZlabel->move(50, 300);

    BLHedit = new QPlainTextEdit(this);
    BLHedit->setFixedSize(800, 50);
    BLHedit->move(50, 450);
    QLabel* BLHlabel = new QLabel("大地坐标(B L H)", this);
    BLHlabel->setFixedSize(200, 50);
    BLHlabel->move(50, 400);

    Cdtransbt = new QPushButton("坐标转换",this);
    Cdtransbt->setFixedSize(200, 50);
    Cdtransbt->move(950, 350);
    connect(Cdtransbt, &QPushButton::clicked, [&]() {
        QString XYZ = XYZedit->toPlainText();
        QString BLH = BLHedit->toPlainText();
        if (!XYZ.isEmpty() && BLH.isEmpty())
        {
            QByteArray bytet = XYZ.toUtf8();
            char* tchar = bytet.data();
            double xyz[3] = {}, blh[3] = {};
            sscanf(tchar, "%lf %lf %lf", &xyz[0], &xyz[1], &xyz[2]);
            xyztoblh_t(xyz[0], xyz[1], xyz[2], blh);
            QString plaintext;
            BLHedit->setPlainText(plaintext.sprintf("%.10lf %.10lf %.10lf" ,blh[0], blh[1], blh[2]));
        }
        else if (XYZ.isEmpty() && !BLH.isEmpty()) {
            QByteArray bytet = BLH.toUtf8();
            char* tchar = bytet.data();
            double xyz[3] = {}, blh[3] = {};
            sscanf(tchar, "%lf %lf %lf", &blh[0], &blh[1], &blh[2]);
            blhtoxyz_t(blh[0], blh[1], blh[2], xyz);
            QString plaintext;
            XYZedit->setPlainText(plaintext.sprintf("%.10lf %.10lf %.10lf", xyz[0], xyz[1], xyz[2]));
        }
        else
            QMessageBox::critical(this, "坐标输入格式错误", "请输入正确的待转换坐标");
        });

    Cdtransclearbt = new QPushButton("清空坐标转换区", this);
    Cdtransclearbt->setFixedSize(200, 50);
    Cdtransclearbt->move(950, 450);
    connect(Cdtransclearbt, &QPushButton::clicked, [&]() {
        XYZedit->clear();
        BLHedit->clear();
        });
}

#pragma 对流层电离层延迟解算区
void Stage::Trp_ionInit(void) {
    RR = new QPlainTextEdit(this);
    RR->setFixedSize(750, 50);
    RR->move(50, 550);
    QLabel* RRlabel = new QLabel("测站坐标", this);
    RRlabel->setFixedSize(150, 50);
    RRlabel->move(830, 550);
    
    SP = new QPlainTextEdit(this);
    SP->setFixedSize(750, 50);
    SP->move(50, 620);
    QLabel* SPlabel = new QLabel("卫星坐标", this);
    SPlabel->setFixedSize(150, 50);
    SPlabel->move(830, 620);

    IonEdit_a = new QPlainTextEdit(this);
    IonEdit_a->setFixedSize(600, 30);
    IonEdit_a->move(50, 700);
    IonEdit_b = new QPlainTextEdit(this);
    IonEdit_b->setFixedSize(600, 30);
    IonEdit_b->move(50, 735);
    QLabel* Ionlabel_a = new QLabel("电离层参数(ALPHA)", this);
    Ionlabel_a->setFixedSize(200, 30);
    Ionlabel_a->move(680, 700);
    QLabel* Ionlabel_b = new QLabel("电离层参数(BETA)", this);
    Ionlabel_b->setFixedSize(200, 30);
    Ionlabel_b->move(680, 735);

    Trapmodelselect = new QComboBox(this);
    Trapmodelselect->addItem("Saastamonien");
    Trapmodelselect->addItem("Simple Hopfield");
    Trapmodelselect->setFixedSize(200, 50);
    Trapmodelselect->move(950, 550);
    
    QPushButton* Tripmodelbt = new QPushButton("对流层延迟", this);
    Tripmodelbt->setFixedSize(200, 50);
    Tripmodelbt->move(950, 620);
    connect(Tripmodelbt, &QPushButton::clicked, [&]() {
        QString rrstr = RR->toPlainText();
        QString spstr = SP->toPlainText();
        if (rrstr.isEmpty() || spstr.isEmpty())
            QMessageBox::critical(this, "坐标错误", "请输入正确的卫星和测站直角坐标");
        else {
            QByteArray tbyte1 = rrstr.toUtf8();
            QByteArray tbyte2 = spstr.toUtf8();
            char* trr = tbyte1.data(); char* tsp = tbyte2.data();
            double rr[3] = {}, sp[3] = {};
            sscanf(trr, "%lf %lf %lf", &rr[0], &rr[1], &rr[2]);
            sscanf(tsp, "%lf %lf %lf", &sp[0], &sp[1], &sp[2]);
            double trp = 0.0, azel[2] = {};
            getazel_t(sp, rr, azel);
            if (Trapmodelselect->currentText() == QString("Saastamonien"))
                trp = tropmodel_t(rr, azel);
            else if (Trapmodelselect->currentText() == "Simple Hopfield")
                trp = 2.47 / (sin(azel[1]) + 0.0121);
            else
                trp = 0;
            char result[200] = {};
            sprintf(result, "对流层延迟:%.10lf", trp);
            QMessageBox::information(this, "对流层延迟解算结果", QString(result));
        }
        
        });

    QPushButton* Ionmodelbt = new QPushButton("电离层延迟", this);
    Ionmodelbt->setFixedSize(200, 50);
    Ionmodelbt->move(950, 700);
    connect(Ionmodelbt, &QPushButton::clicked, [&]() {
        QString rrstr = RR->toPlainText();
        QString spstr = SP->toPlainText();
        QString ionastr = IonEdit_a->toPlainText();
        QString ionbstr = IonEdit_b->toPlainText();
        gtime_t ut = { UnixTtimes->toPlainText().toLongLong(),UnixTseconds->toPlainText().toDouble() };
        GPSTime gpst = {}; int week;
        gpst.Second = time2gpst_t(ut, week);
        gpst.Week = int(week);
        if (rrstr.isEmpty() || spstr.isEmpty() || ionastr.isEmpty() || ionbstr.isEmpty() ||
            UnixTtimes->toPlainText().isEmpty() || UnixTseconds->toPlainText().isEmpty())
            QMessageBox::critical(this, "坐标、参数或时间错误", "请输入正确的时间、坐标或电离层参数");
        else {
            QByteArray tbyte1 = rrstr.toUtf8();
            QByteArray tbyte2 = spstr.toUtf8();
            QByteArray tbyte3 = ionastr.toUtf8();
            QByteArray tbyte4 = ionbstr.toUtf8();
            char* trr = tbyte1.data(); char* tsp = tbyte2.data();
            char* tiona = tbyte3.data(); char* tionb = tbyte4.data();
            double rr[3] = {}, sp[3] = {}, ion[8] = {};
            sscanf(trr, "%lf %lf %lf", &rr[0], &rr[1], &rr[2]);
            sscanf(tsp, "%lf %lf %lf", &sp[0], &sp[1], &sp[2]);
            double dion = 0.0, azel[2] = {};
            getazel_t(sp, rr, azel);
            
            if (strstr(tiona, "D"))
                charreplace_t(tiona, 'D', 'E');
            
            if(strstr(tionb,"D"))
                charreplace_t(tionb, 'D', 'E');
            
            sscanf(tiona, "%lf %lf %lf %lf", &ion[0], &ion[1], &ion[2], &ion[3]);
            sscanf(tionb, "%lf %lf %lf %lf", &ion[4], &ion[5], &ion[6], &ion[7]);
            dion=ionmodel_t(gpst, ion, rr, azel);
             
            char result[200] = {};
            sprintf(result, "电离层延迟:%.10lf", dion);
            QMessageBox::information(this, "电离层延迟解算结果", QString(result));
        }
        
        });
    
}

void Stage::Stage_resize(void) {
    QScreen* screen = QGuiApplication::primaryScreen();
    int w = screen->size().width();
    int h = screen->size().height();
    double wb = w / 2520.0;//横比例因子 
    double hb = h / 1680.0;//纵比例因子
    QList<QPushButton*>btnlist = this->findChildren<QPushButton*>();
    for (QPushButton* btn : btnlist) {
        QFont font;
        font.setPixelSize(round(24 * wb));
        btn->setFont(font);
        btn->setFixedSize(btn->size().width() * wb, btn->size().height() * hb);
        btn->move(btn->pos().x() * wb, btn->pos().y() * hb);
    }
    QList<QLabel*>lblist = this->findChildren<QLabel*>();
    for (QLabel* lb : lblist) {
        QFont font;
        font.setPixelSize(round(24 * wb));
        lb->setFont(font);
        lb->setFixedSize(lb->size().width() * wb, lb->size().height() * hb);
        lb->move(lb->pos().x() * wb, lb->pos().y() * hb);
    }
    QList<QComboBox*>cboxlist = this->findChildren<QComboBox*>();
    for (QComboBox* cbox : cboxlist) {
        QFont font;
        font.setPixelSize(round(24 * wb));
        cbox->setFont(font);
        cbox->setFixedSize(cbox->size().width() * wb, cbox->size().height() * hb);
        cbox->move(cbox->pos().x() * wb, cbox->pos().y() * hb);
    }
    QList<QPlainTextEdit*>ptelist = this->findChildren<QPlainTextEdit*>();
    for (QPlainTextEdit* pte : ptelist) {
        QFont font;
        font.setPixelSize(round(24 * wb));
        pte->setFont(font);
        pte->setFixedSize(pte->size().width() * wb, pte->size().height() * hb);
        pte->move(pte->pos().x() * wb, pte->pos().y() * hb);
    }
    this->setFixedSize(1200 * wb, 800 * hb);
}

#pragma 子窗口构造函数
Stage::Stage(QWidget *parent)
    : QMainWindow(parent)
{
    this->setWindowTitle("中间值解算");
    TimetransInit();
    CdTransInit();
    Trp_ionInit();
    Stage_resize();
}

#pragma 子窗口析构函数
Stage::~Stage()
{}
