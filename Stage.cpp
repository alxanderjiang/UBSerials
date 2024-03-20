#include "Stage.h"

#pragma �ṹ��ͺ궨��

#define SECOND 315964800
//����Unixʱ�ṹ
struct gtime_t {
    time_t time;
    double second;
};
//����GPSʱ�ṹ
struct GPSTime {
    unsigned short Week;
    double Second;
};
//����ͨ�ü�ʱ�ṹ
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
#define es2 0.00669437999013//��һƫ���ʵ�ƽ��
#define e22 0.006739496742227//�ڶ�ƫ���ʵ�ƽ��
//#define es2 0.0818191910428*0.0818191910428//��һƫ���ʵ�ƽ��
//#define e22 0.0820944381519*0.0820944381519//�ڶ�ƫ���ʵ�ƽ��
#define clight 299792458.0
#pragma �Ӵ��ڵ����������㺯���ع�

struct gtime_t com2unixtime_t(struct COMMONTIME t0) {
    struct gtime_t t;
    int i = 0;
    //��ʼ����ͨ��ʱ��Unixtime���ʱ��Days
    int Days = 0;
    int doy[] = { 1,32,60,91,121,152,182,213,244,274,305,335 };
    Days = (t0.Year - 1970) * 365 + (t0.Year - 1969) / 4 + doy[t0.Month - 1] + t0.Day - 2 + (t0.Year % 4 == 0 && t0.Month >= 3 ? 1 : 0);//��ʦ�ṩ�㷨����2000����ǰʱ����ֳ���ʱ������
    t.time = Days * 86400 + t0.Hour * 3600 + t0.Minute * 60 + floor(t0.Second);
    t.second = t0.Second - floor(t0.Second);
    return  t;
}
//GPSʱתUnix time
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
//unixʱתͨ��ʱ
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
//ֱ��ת���
void xyztoblh_t(double x, double y, double z, double* str) {
    double tb, h, b, l, n;
    double temp1, temp2;
    double m1, m2, m3, m4;
    l = acos(x / sqrt(pow(x, 2) + pow(y, 2)));
    m1 = 1 / sqrt(pow(x, 2) + pow(y, 2));
    m2 = ea * es2;
    m3 = 1 - es2;
    temp1 = z / sqrt(pow(x, 2) + pow(y, 2));
    //����Ϊ����
    do {
        temp2 = temp1;
        temp1 = m1 * (z + m2 * temp1 / sqrt(1 + m3 * pow(temp1, 2)));
    } while (temp1 - temp2 > 1e-14);
    tb = temp1; b = atan(tb);
    n = ea / sqrt(1 - es2 * pow(sin(b), 2));
    h = sqrt(pow(x, 2) + pow(y, 2)) / cos(b) - ea / sqrt(1 - es2 * pow(sin(b), 2));
    if (y < 0) l = -l;
    str[0] = b * 180 / pi, str[1] = l * 180 / pi, str[2] = h;//������
    //printf("�������(%.4lf,%.4lf,%.4lf)\n",str[0],str[1],str[2]);
}
//���תֱ��
void blhtoxyz_t(double b, double l, double h, double* str) {
    double x, y, z;
    double n;
    b = b * pi / 180; l = l * pi / 180;
    n = ea / sqrt(1 - es2 * pow(sin(b), 2));
    x = (n + h) * cos(b) * cos(l);
    y = (n + h) * cos(b) * sin(l);
    z = (n * (1 - es2) + h) * sin(b);
    str[0] = x, str[1] = y, str[2] = z;//������
    //printf("�ռ�ֱ������(%.10lf,%.10lf,%.10lf)\n",x,y,z);
}
//�����
void matx_t(double* A, int rowa, int cola, double* B, int rowb, int colb, double* C) {
    if (cola != rowb) { printf("���󲻿ɳ�"); return; }
    int i, j, k, t;
    for (i = 0; i < rowa; i++)
        for (j = 0; j < colb; j++) {
            for (k = i * cola, t = j; k < i * cola + cola; k++, t += colb)
                C[i * colb + j] += A[k] * B[t];
        }
}
//���º�������Ϊ�������Ƿ���Ǻ͸߶Ƚ�
void getazel_t(const double* rs, const double* rr, double* azel) {
    //�Ӳ����������Ǻͽ��ջ�����ͬʱת��Ϊ�������
    //double ion[3]={},pos[3]={};
    double rsblh[3]; xyztoblh_t(rs[0], rs[1], rs[2], rsblh);
    double rrblh[3]; xyztoblh_t(rr[0], rr[1], rr[2], rrblh);//�����
    //��ע��xyztoblh��������ֵ�ĸ�ʽ�ǽǶ��ƻ��ǻ����ƣ�������Ϊ�Ƕ���
    double d = sqrt(pow(rs[0] - rr[0], 2) + pow(rs[1] - rr[1], 2) + pow(rs[2] - rr[2], 2));
    double e1, e2, e3, e[3];

    e1 = (rs[0] - rr[0]) / d; e2 = (rs[1] - rr[1]) / d; e3 = (rs[2] - rr[2]) / d;
    e[0] = rs[0] - rr[0]; e[1] = rs[1] - rr[1]; e[2] = rs[2] - rr[2];
    double B = rrblh[0] * pi / 180, L = rrblh[1] * pi / 180;//��վ�Ĵ�����꣬ת��Ϊ�������Լ������Ǻ���

    //��������ϵתվ������ϵ
    double E, N, U, ER[3] = {};
    double H[9] = { -sin(L),cos(L),0,
                -sin(B) * cos(L),-sin(B) * sin(L),cos(B),
                cos(B) * cos(L),cos(B) * sin(L),sin(B) };

    matx_t(H, 3, 3, e, 3, 1, ER);
    E = ER[0]; N = ER[1]; U = ER[2];

    //���Ǹ߶Ƚ�/����ǵļ���
    double az, el;
    az = atan2(E, N);
    el = asin(U / d);
    azel[0] = az;
    azel[1] = el;
}
//���º�������Ϊ���������ӳ�
double ionmodel_t(GPSTime t, const double* ion, const double* pos, const double* azel) {
    //�Ӳ�����ȡ����Ҫ����ֵ
    double az = azel[0];
    double el = azel[1];
    double rrblh[3]; xyztoblh_t(pos[0], pos[1], pos[2], rrblh);
    //����������Ľ�
    double Phi = 0.0137 / ((el / pi) + 0.11) - 0.022;
    //�������㴩�̵��γ��phi1
    double phi1, phiu;
    phiu = rrblh[0] / 180.0;
    phi1 = phiu + Phi * cos(az);
    if (phi1 > 0.416) phi1 = 0.416;
    else if (phi1 < -0.416) phi1 = -0.416;
    //�������㴩�̵㾭��lamda1
    double lamda1, lamdau;
    lamdau = rrblh[1] / 180.0;
    lamda1 = lamdau + Phi * sin(az) / cos(phi1 * pi);
    //�������㴩�̵�ĵش�γ��phim
    double phim;
    phim = phi1 + 0.064 * cos((lamda1 - 1.617) * pi);
    //�������㴩�̵�ĵ���ʱ��localtime
    double localtime;
    localtime = 43200 * lamda1 + t.Second;//���㵱��ʱ����GPS������Ϊ��׼��
    localtime = localtime - floor(localtime / 86400.0) * 86400;//�۳������������õ�һ���ڵĵط�ʱ����
    //���������ӳٵķ���A1
    double A1;
    A1 = ion[0] + phim * (ion[1] + phim * (ion[2] + phim * ion[3]));
    if (A1 < 0) A1 = 0;
    //���������ӳٵ�����P1
    double P1;
    P1 = ion[4] + phim * (ion[5] + phim * (ion[6] + phim * ion[7]));
    if (P1 < 72000) P1 = 72000;
    //���������ӳ���λX1
    double X1;
    X1 = 2 * pi * (localtime - 50400) / P1;
    //������б����F
    double F;
    F = 1.0 + 16.0 * pow((0.53 - el / pi), 3);

    //ģ�Ͳ���������ϣ��������ģ�ͼ��������ӳ�IL1GPS
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
    //�Ӳ����ж�ȡ���Ǹ߶ȽǷ�λ��
    double az = azel[0], el = azel[1];
    //�Ӳ����ж�ȡ��������
    double rblh[3] = {};
    xyztoblh_t(pos[0], pos[1], pos[2], rblh);
    //�������뾶�͵����߶�
    double Re = 6378000.0, hion = 375000.0;
    //�û��ʹ��̵�ĵ����Ž�
    double angel = pi / 2 - el - asin(Re / (Re + hion) * cos(el));
    //����㴩�̵�ĵ���γ�ȡ�������
    double phiM = asin(sin(rblh[0]) * cos(angel) + cos(rblh[0]) * sin(angel) * cos(az));
    double lambdaM = rblh[1] + asin(sin(angel) * sin(az) / cos(phiM));
    //������������
    double A4, x = phiM / pi;
    A4 = ion[4] + ion[5] * x + ion[6] * x * x + ion[7] * x * x * x;
    A4 = (A4 >= 172800 ? 172800 : A4);
    A4 = (A4 < 72000 ? 72000 : A4);
    //���������ӳ��������ߵķ���
    double A2;
    A2 = ion[0] + ion[1] * x + ion[2] * x * x + ion[3] * x * x * x;
    A2 = (A2 < 0 ? 0 : A2);
    //���̵�ط�ʱ
    double t = (tobs.Second - 14 + lambdaM * 43200.0 / pi);
    //��ֱ�ӳٸ���
    double Iz;
    if (abs(t - 50400) >= A4 / 4)
        Iz = 5e-9;
    else
        Iz = 5e-9 + A2 * cos(2 * pi * (t - 50400) / A4);
    //B1�ź��ӳ�
    double IzB1I = 1.0 / sqrt(1 - (Re / (Re + hion) * cos(el)) * (Re / (Re + hion) * cos(el))) * Iz;
    //����
    return IzB1I * clight;
}
//���º�������Ϊ�������ӳټ���
double tropmodel_t(const double* pos, const double* azel) {
    //��ֱ������ת��Ϊ�������
    double posblh[3]; xyztoblh_t(pos[0], pos[1], pos[2], posblh);
    //���ڵ����ϣ��������ӳٹ���
    if (posblh[2] < -100.0 || 1E4 < posblh[2] || azel[1] <= 0) return 0.0;

    double humi = 0.7;
    double h = posblh[2], b = posblh[0] * pi / 180.0;//��Ϊ��ͷ�ļ�������ת����������Ϊ�Ƕ�ֵ�����Լ���ǰ��Ҫ��ԭ
    if (posblh[2] < 0.0) h = 0.0;//����̹߳��㴦��

    double T = 15.0 - 6.5 * 1e-3 * h + 273.16;
    double e = 6.108 * humi * exp((17.15 * T - 4684.0) / (T - 38.45));
    double p = 1013.25 * pow((1 - 2.2557e-5 * h), 5.2568);
    double z = pi / 2.0 - azel[1];
    double trph = 0.0022768 * p / (cos(z) * (1.0 - 0.00266 * cos(2 * b) - 0.00028 * h / 1000.0));
    double trpw = 0.002277 * (1255.0 / T + 0.05) * e / cos(z);
    double trp = trph + trpw;
    return trp;
}
//���º�������Ϊ�ַ����ض��ַ��滻(c->b)
void charreplace_t(char* str, char c, char b) {
    for (int i = 0; i < strlen(str); i++)
        if (str[i] == c)
            str[i] = b;
}

#pragma ʱ��ת����
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
    QLabel* text1 = new QLabel("��", this);
    text1->move(150, 210);
    Monthedit->setFixedSize(100, 50);
    Monthedit->move(200, 200);
    QLabel* text2 = new QLabel("��", this);
    text2->move(300, 210);
    Dayedit->setFixedSize(100, 50);
    Dayedit->move(350, 200);
    QLabel* text3 = new QLabel("��", this);
    text3->move(450, 210);
    Houredit->setFixedSize(100, 50);
    Houredit->move(500, 200);
    QLabel* text4 = new QLabel("ʱ", this);
    text4->move(600, 210);
    Minuteedit->setFixedSize(100, 50);
    Minuteedit->move(650, 200);
    QLabel* text5 = new QLabel("��", this);
    text5->move(750, 210);
    Secondsedit->setFixedSize(200, 50);
    Secondsedit->move(800, 200);
    QLabel* text6 = new QLabel("��", this);
    text6->move(1000, 210);
    
    
    Timetransbt = new QPushButton("ʱ��ת��", this);
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
            QMessageBox::critical(this,"ʱ�������ʽ����", "��������ȷ�Ĵ�ת��ʱ��");
        });

    Timetransclearbt = new QPushButton("���ʱ��ת����", this);
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

#pragma ����ת����
void Stage::CdTransInit(void) {
    XYZedit = new QPlainTextEdit(this);
    XYZedit->setFixedSize(800, 50);
    XYZedit->move(50, 350);
    QLabel* XYZlabel = new QLabel("ֱ������(X Y Z)", this);
    XYZlabel->setFixedSize(200, 50);
    XYZlabel->move(50, 300);

    BLHedit = new QPlainTextEdit(this);
    BLHedit->setFixedSize(800, 50);
    BLHedit->move(50, 450);
    QLabel* BLHlabel = new QLabel("�������(B L H)", this);
    BLHlabel->setFixedSize(200, 50);
    BLHlabel->move(50, 400);

    Cdtransbt = new QPushButton("����ת��",this);
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
            QMessageBox::critical(this, "���������ʽ����", "��������ȷ�Ĵ�ת������");
        });

    Cdtransclearbt = new QPushButton("�������ת����", this);
    Cdtransclearbt->setFixedSize(200, 50);
    Cdtransclearbt->move(950, 450);
    connect(Cdtransclearbt, &QPushButton::clicked, [&]() {
        XYZedit->clear();
        BLHedit->clear();
        });
}

#pragma �����������ӳٽ�����
void Stage::Trp_ionInit(void) {
    RR = new QPlainTextEdit(this);
    RR->setFixedSize(750, 50);
    RR->move(50, 550);
    QLabel* RRlabel = new QLabel("��վ����", this);
    RRlabel->setFixedSize(150, 50);
    RRlabel->move(830, 550);
    
    SP = new QPlainTextEdit(this);
    SP->setFixedSize(750, 50);
    SP->move(50, 620);
    QLabel* SPlabel = new QLabel("��������", this);
    SPlabel->setFixedSize(150, 50);
    SPlabel->move(830, 620);

    IonEdit_a = new QPlainTextEdit(this);
    IonEdit_a->setFixedSize(600, 30);
    IonEdit_a->move(50, 700);
    IonEdit_b = new QPlainTextEdit(this);
    IonEdit_b->setFixedSize(600, 30);
    IonEdit_b->move(50, 735);
    QLabel* Ionlabel_a = new QLabel("��������(ALPHA)", this);
    Ionlabel_a->setFixedSize(200, 30);
    Ionlabel_a->move(680, 700);
    QLabel* Ionlabel_b = new QLabel("��������(BETA)", this);
    Ionlabel_b->setFixedSize(200, 30);
    Ionlabel_b->move(680, 735);

    Trapmodelselect = new QComboBox(this);
    Trapmodelselect->addItem("Saastamonien");
    Trapmodelselect->addItem("Simple Hopfield");
    Trapmodelselect->setFixedSize(200, 50);
    Trapmodelselect->move(950, 550);
    
    QPushButton* Tripmodelbt = new QPushButton("�������ӳ�", this);
    Tripmodelbt->setFixedSize(200, 50);
    Tripmodelbt->move(950, 620);
    connect(Tripmodelbt, &QPushButton::clicked, [&]() {
        QString rrstr = RR->toPlainText();
        QString spstr = SP->toPlainText();
        if (rrstr.isEmpty() || spstr.isEmpty())
            QMessageBox::critical(this, "�������", "��������ȷ�����ǺͲ�վֱ������");
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
            sprintf(result, "�������ӳ�:%.10lf", trp);
            QMessageBox::information(this, "�������ӳٽ�����", QString(result));
        }
        
        });

    QPushButton* Ionmodelbt = new QPushButton("������ӳ�", this);
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
            QMessageBox::critical(this, "���ꡢ������ʱ�����", "��������ȷ��ʱ�䡢������������");
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
            sprintf(result, "������ӳ�:%.10lf", dion);
            QMessageBox::information(this, "������ӳٽ�����", QString(result));
        }
        
        });
    
}

void Stage::Stage_resize(void) {
    QScreen* screen = QGuiApplication::primaryScreen();
    int w = screen->size().width();
    int h = screen->size().height();
    double wb = w / 2520.0;//��������� 
    double hb = h / 1680.0;//�ݱ�������
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

#pragma �Ӵ��ڹ��캯��
Stage::Stage(QWidget *parent)
    : QMainWindow(parent)
{
    this->setWindowTitle("�м�ֵ����");
    TimetransInit();
    CdTransInit();
    Trp_ionInit();
    Stage_resize();
}

#pragma �Ӵ�����������
Stage::~Stage()
{}
