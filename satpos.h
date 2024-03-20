/*文件名：satpos*/
/*功能：单点定位相关函数*/
/*对流层，电离层延迟解算、卫星钟差计算、卫星方位角高度角计算、卫星位置计算、单点定位*/
#include <stdio.h>
#include <math.h>
#include <string.h>
#include"coordinatetrans.h"
#include"binaryread.h"
//#include"class4.h"
#include"matprocess.h"
#include <math.h>
//以下参数基于WGS-84坐标系
#define OMGE 7.2921151467E-5
#define GM 3.9860047e14
using namespace std;
//#define f 1.0/298.257223563
//#define a 6378137.0
#define pi 3.1415926535897932384626433832795
#define Fc -4.442807633e-10//相对论效应改正参数
//以下参数基于CGCS2000坐标系
#define OMGE_BDS 7.2921150E-5
#define GM_BDS 3.986004418E14
#define Fc_BDS -2*sqrt(GM_BDS)/clight/clight



/*电离层、对流层、卫星高度角方向角计算函数*/

//以下函数功能为从文件中读取电离层参数
int getion(const char* filename, double* ion) {
    double iona[8] = {}, ionb[8] = {};
    FILE* fp = fopen(filename, "r");
    char ch; char fullstr[100]; int flag = 0;
    while (!feof(fp)) {
        fgets(fullstr, 100, fp);
        char stra[] = { "ALPHA" };
        char strb[] = { "BETA" };
        if (strstr(fullstr, stra))
            sscanf(fullstr, "%lfD%lf %lfD%lf %lfD%lf %lfD%lf", &iona[0], &iona[1], &iona[2], &iona[3], &iona[4], &iona[5], &iona[6], &iona[7]);
        if (strstr(fullstr, strb)) {
            sscanf(fullstr, "%lfD%lf %lfD%lf %lfD%lf %lfD%lf", &ionb[0], &ionb[1], &ionb[2], &ionb[3], &ionb[4], &ionb[5], &ionb[6], &ionb[7]);
            flag = 1;
            break;
        }
    }
    if (flag == 0) return 0;
    fclose(fp);
    ion[0] = iona[0] * (pow(10, iona[1]));
    ion[1] = iona[2] * (pow(10, iona[3]));
    ion[2] = iona[4] * (pow(10, iona[5]));
    ion[3] = iona[6] * (pow(10, iona[7]));
    ion[4] = ionb[0] * (pow(10, ionb[1]));
    ion[5] = ionb[2] * (pow(10, ionb[3]));
    ion[6] = ionb[4] * (pow(10, ionb[5]));
    ion[7] = ionb[6] * (pow(10, ionb[7]));
    return 1;
}
//以下函数为从二进制消息中读取电离层延迟参数
void getion(unsigned char* fullodata, double* ion, breport epoch) {
    int start = epoch.start, end = epoch.end;
    ion[0] = bit2double(fullodata + start + 24);
    ion[1] = bit2double(fullodata + start + 24 + 8);
    ion[2] = bit2double(fullodata + start + 24 + 16);
    ion[3] = bit2double(fullodata + start + 24 + 24);
    ion[4] = bit2double(fullodata + start + 24 + 32);
    ion[5] = bit2double(fullodata + start + 24 + 40);
    ion[6] = bit2double(fullodata + start + 24 + 48);
    ion[7] = bit2double(fullodata + start + 24 + 56);
}
void getion(unsigned char* fullodata, double* ion, breport epoch,const char* OEM7) {
    int start = epoch.start, end = epoch.end;
    ion[0] = bit2double(fullodata + start + 28);
    ion[1] = bit2double(fullodata + start + 28 + 8);
    ion[2] = bit2double(fullodata + start + 28 + 16);
    ion[3] = bit2double(fullodata + start + 28 + 24);
    ion[4] = bit2double(fullodata + start + 28 + 32);
    ion[5] = bit2double(fullodata + start + 28 + 40);
    ion[6] = bit2double(fullodata + start + 28 + 48);
    ion[7] = bit2double(fullodata + start + 28 + 56);
}

/*解算所得的卫星轨道状态结构体（位置、速度、钟差、钟速）*/
typedef struct {
    char country = 'G';//系统归属，缺省值为GPS
    int name = 0;//PRN编号
    int statu = 0;//0=未解出，1=已解出
    double xyzt[4] = {};//卫星位置、钟差
    double dotxyzt[4] = {};//卫星速度、钟速
    double ion = 0.0;//电离层延迟
    double trp = 0.0;//对流层延迟
    gtime_t tobs;//观测时间
}Satxyzt;
Satxyzt BDS_xzy[65] = {};//BDS卫星轨道状态
Satxyzt GPS_xyz[36] = {};//GPS卫星轨道状态
double azel_BDS[65][2] = {};//BDS卫星高度方向角
double azel_GPS[36][2] = {};//GPS卫星高度方向角

//以下函数功能为计算卫星方向角和高度角
void getazel(const double* rs, const double* rr, double* azel) {
    //从参数接受卫星和接收机坐标同时转化为大地坐标
    //double ion[3]={},pos[3]={};
    double rsblh[3]; xyztoblh(rs[0], rs[1], rs[2], rsblh);
    double rrblh[3]; xyztoblh(rr[0], rr[1], rr[2], rrblh);//错误点
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

    matx(H, 3, 3, e, 3, 1, ER);
    E = ER[0]; N = ER[1]; U = ER[2];

    //卫星高度角/方向角的计算
    double az, el;
    az = atan2(E, N);
    el = asin(U / d);
    azel[0] = az;
    azel[1] = el;
}
//以下函数功能为计算电离层延迟
double ionmodel(GPSTime t, const double* ion, const double* pos, const double* azel) {
    //从参数中取出需要的数值
    double az = azel[0];
    double el = azel[1];
    double rrblh[3]; xyztoblh(pos[0], pos[1], pos[2], rrblh);
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

double ionmodel_BDS(GPSTime tobs, const double* ion, const double* pos, const double* azel) {
    //从参数中读取卫星高度角方位角
    double az = azel[0], el = azel[1];
    //从参数中读取地理坐标
    double rblh[3] = {};
    xyztoblh(pos[0], pos[1], pos[2], rblh);
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
    double IzB1I = 1.0 / sqrt(1 - (Re / (Re + hion) * cos(el)) * (Re / (Re + hion) * cos(el)) ) * Iz;
    //秒归距
    return IzB1I * clight;
}
//以下函数功能为对流层延迟计算
double tropmodel(const double* pos, const double* azel) {
    //将直角坐标转换为大地坐标
    double posblh[3]; xyztoblh(pos[0], pos[1], pos[2], posblh);
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

/*卫星高度角、方向角、电离层延迟、对流层延迟计算函数结束*/

/*卫星位置计算函数开始*/

//字符串转数字
int char2int(char c) {
    if (c >= '0' && c <= '9')
        return c - '0';
    else
        return 0;
}

//读取卫星星历数据

void readmessagefile(int name, const char* filename, double* data, int* snum) {

    FILE* fp = fopen(filename, "r");
    char ch; char fullstr[100];

    while (1) {
        fgets(fullstr, 100, fp);
        if (strstr(fullstr, "END OF HEADER"))
            break;
    }
    //找到目标卫星数据在文件中的位置
    while (1) {
        fgets(fullstr, 100, fp);
        char index[3] = { fullstr[0],fullstr[1],fullstr[2] };
        int fhead = 0; sscanf(index, "%d", &fhead);
        if (fhead == name)
            break;
    }

    int line = 1;
    while (1) {
        double dataa[8] = {}, datab[8] = {};
        //fgets(fullstr,100,fp);
        if (line == 1) {
            char tc;
            if (strstr(fullstr, "G"))
                sscanf(fullstr, "G%d %d %d %d %d %d %lf %lf%c%lf %lf%c%lf %lf%c%lf", &snum[0], &snum[1], &snum[2], &snum[3], &snum[4], &snum[5], &snum[6], &dataa[0], &tc, &datab[0], &dataa[1], &tc, &datab[1], &dataa[2], &tc, &datab[2]);
            else
                sscanf(fullstr, "%d %d %d %d %d %d %lf %lf%c%lf %lf%c%lf %lf%c%lf", &snum[0], &snum[1], &snum[2], &snum[3], &snum[4], &snum[5], &snum[6], &dataa[0], &tc, &datab[0], &dataa[1], &tc, &datab[1], &dataa[2], &tc, &datab[2]);
            data[1] = dataa[0] * pow(10, datab[0]);
            data[2] = dataa[1] * pow(10, datab[1]);
            data[3] = dataa[2] * pow(10, datab[2]);
        }
        else {
            fgets(fullstr, 100, fp);
            char tc;
            sscanf(fullstr, "%lf%c%lf %lf%c%lf %lf%c%lf %lf%c%lf", &dataa[0], &tc, &datab[0], &dataa[1], &tc, &datab[1], &dataa[2], &tc, &datab[2], &dataa[3], &tc, &datab[3]);
            data[0 + 4 * (line - 1)] = dataa[0] * pow(10, datab[0]);
            data[1 + 4 * (line - 1)] = dataa[1] * pow(10, datab[1]);
            data[2 + 4 * (line - 1)] = dataa[2] * pow(10, datab[2]);
            data[3 + 4 * (line - 1)] = dataa[3] * pow(10, datab[3]);
        }
        line++;

        if (line == 9) break;
    }
    data[0] = snum[0];
    fclose(fp);
};

//将卫星星历数据转移到结构体中
eph_t getsate(double* data) {
    eph_t s;
    s.af0 = data[1];
    s.af1 = data[2];
    s.af2 = data[3];
    s.crs = data[5];
    s.deln = data[6];
    s.M0 = data[7];
    s.cuc = data[8];
    s.e = data[9];
    s.cus = data[10];
    s.A = data[11] * data[11];
    s.toes = data[12];
    s.cic = data[13];
    s.OMG0 = data[14];
    s.cis = data[15];
    s.i0 = data[16];
    s.crc = data[17];
    s.omg = data[18];
    s.OMGD = data[19];
    s.idot = data[20];
    s.week = data[22];
    s.svh = data[25];
    s.tgd = data[26];
    s.N = 0;
    s.toc = s.toe = gpst2time(s.week, s.toes);
    return s;
}

struct gtime_t com2unixtime(struct COMMONTIME t0) {
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
//从RINEX文件中读取观测时间和伪距
int gettobsandrho(char* filename, char country, int name, gtime_t& t, const double& rho, int rank) {
    //在filename对应的观测文件中查找country国家的name号卫星，并读取它在这个观测文件中的第rank个历元的伪距和观测时间

    FILE* fp = fopen(filename, "r");
    char fullstr[100];
    char prn[5] = { country };
    if (name <= 9) {
        prn[1] = '0';
        prn[2] = '0' + name;
    }
    else {
        prn[1] = '0' + name / 10;
        prn[2] = '0' + name % 10;
    }

    while (1) {
        fgets(fullstr, 100, fp);
        if (strstr(fullstr, "END OF HEADER"))
            break;
    }
    struct COMMONTIME temp;
    int rankflag = 0;
    while (1) {
        fgets(fullstr, 100, fp);
        char tflag[5] = { ">" };
        if (strstr(fullstr, tflag)) rankflag++;
        if (rankflag == rank) {
            sscanf(fullstr, "> %d %d %d %d %d %lf", &temp.Year, &temp.Month, &temp.Day, &temp.Hour, &temp.Minute, &temp.Second);
            break;
        }
    }
    //sscanf(fullstr,"> %d %d %d %d %d %lf",&temp.Year,&temp.Month,&temp.Day,&temp.Hour,&temp.Minute,&temp.Second);
    t = com2unixtime(temp);
    int findflage = 0;
    while (1) {
        if (fgets(fullstr, 100, fp) == NULL)
            break;
        if (strstr(fullstr, prn)) {
            findflage = 1;
            char s[3];
            sscanf(fullstr, "%s %lf", &s, &rho);
            break;
        }
    }
    fclose(fp);
    if (findflage == 0)
        printf("未接收到%s卫星信号", prn);

    return findflage;
}
//直接应用伪距数组(结构体)获取伪距
int gettobsandrho(double* R, char country, int name, double& rho) {
    if (R[name] < 200000) { printf("未接收到%c%02d卫星信号\n", country, name); return 0; }//无对应伪距
    else {
        rho = R[name];
    }
    return 1;
}
//计算卫星钟差
double getdts(double* data, double t0, double t) {
    double dts;
    dts = data[1] + data[2] * (t - t0) + data[3] * (t - t0) * (t - t0);
    return dts;
}
double getdts(eph_t s, gtime_t rt) {
    double dts;
    dts = s.af0 + s.af1 * double(rt.time + rt.second - s.toc.time - s.toc.second) + s.af2 * double(rt.time + rt.second - s.toc.time - s.toc.second) * double(rt.time + rt.second - s.toc.time - s.toc.second);
    return dts;
}
double getdts(eph_bds2 s, gtime_t rt) {
    double dts; gtime_t st0 = {};
    st0 = gpst2time(s.week, s.toe);
    dts= s.af0 + s.af1 * double(rt.time + rt.second - st0.time - st0.second) + s.af2 * double(rt.time + rt.second - st0.time - st0.second) * double(rt.time + rt.second - st0.time - st0.second);
    return dts;
}
//北斗卫星位置计算函数
void BDSsatpos(eph_bds2 satellite, char country, int name, double rho, double* xyz, double* dotxyz, gtime_t rt, double& tdts, double& tdtss) {
    
    if (satellite.statu == EPHNO)
    {
        xyz[0] = xyz[1] = xyz[2] = 0;
        dotxyz[0] = dotxyz[1] = dotxyz[2] = 0;
        tdts = tdtss = 0;
        return;
    }//星历不存在，不参与计算
    
    struct COMMONTIME comtime = {};

    gtime_t tobs = rt;//观测时间
    tobs.time -= 14;//GPS秒与北斗秒差异
    gtime_t toe = gpst2time(satellite.week, satellite.toe);//星历参考时间
    gtime_t toc = gpst2time(satellite.week, satellite.toc);

    //计算卫星轨道长半轴
    double A = satellite.A;

    //计算平均运动角速度
    double n0 = sqrt(GM_BDS / A / A / A);//GM为地球引力常数

    //计算相对星历参考历元的时间

    double t = tobs.time + tobs.second - rho / clight - getdts(satellite, tobs);
    t = t + satellite.tgd1;
    double tk = t - toe.time - toe.second;

    if (tk > 302400) tk = tk - 604800;
    else if (tk < -302400) tk = tk + 604800;
    else tk = tk;
    //对平均运动角速度进行改正Mk
    double n = satellite.deltN + n0;
    //计算平进角点
    double Mk = satellite.M0 + n * tk;
    //计算偏近角点
    double Ek = Mk;
    double Ek1;
    while (1) {
        //Ek1=Ek-(Ek-satellite.e*sin(Ek)-Mk)/(1-satellite.e*cos(Ek));
        Ek1 = Mk + satellite.e * sin(Ek);
        if (abs(Ek1 - Ek) < 1e-13) break;
        Ek = Ek1;
    }
    Ek = Ek1;
    //计算真近点角vk
    double vk = atan2(sqrt(1 - satellite.e * satellite.e) * sin(Ek) / (1 - satellite.e * cos(Ek)), (cos(Ek) - satellite.e) / (1 - satellite.e * cos(Ek)));
    //计算升交点角距Phik
    double Phik = vk + satellite.omg;
    //计算二阶调和改正数
    double deltauk = satellite.cus * sin(2 * Phik) + satellite.cuc * cos(2 * Phik);
    double deltark = satellite.crs * sin(2 * Phik) + satellite.crc * cos(2 * Phik);
    double deltaik = satellite.cis * sin(2 * Phik) + satellite.cic * cos(2 * Phik);
    //计算改正的升交角距
    double uk = Phik + deltauk;
    //计算改正的向径
    double rk = A * (1 - satellite.e * cos(Ek)) + deltark;
    //计算改正的轨道倾角
    double ik = satellite.I0 + deltaik + satellite.idot * tk;
    //计算卫星在轨道平面上的位置
    double xk1 = rk * cos(uk);
    double yk1 = rk * sin(uk);
    //计算经过改正的升交点经度

    double Omegak = satellite.OMG0 + (satellite.OMGD - OMGE_BDS) * tk - OMGE_BDS * satellite.toe;
    double Omegak1 = satellite.OMG0 + satellite.OMGD * tk - OMGE_BDS * satellite.toe;
    //计算卫星在地心地固坐标系下位置
    double xk = xk1 * cos(Omegak) - yk1 * cos(ik) * sin(Omegak);
    double yk = xk1 * sin(Omegak) + yk1 * cos(ik) * cos(Omegak);
    double zk = yk1 * sin(ik);
    //IGSO/MEO卫星位置计算
    if (name > 5 && name < 59) {
        xyz[0] = xk;
        xyz[1] = yk;
        xyz[2] = zk;
    }
    //GEO卫星位置计算
    else {
        double Ometk = OMGE_BDS * tk, f = -5.0 / 180 * pi;
        double Rx[9] = { 1,0,0,  0,cos(f),sin(f),  0,-sin(f),cos(f) };
        double Rz[9] = { cos(Ometk),sin(Ometk),0,  -sin(Ometk),cos(Ometk),0,  0,0,1 };//自转改正在外部进行
        double xyzgk[3] = {}, xyzr[3] = {};
        xyzgk[0] = xk1 * cos(Omegak1) - yk1 * cos(ik) * sin(Omegak1);
        xyzgk[1] = xk1 * sin(Omegak1) + yk1 * cos(ik) * cos(Omegak1);
        xyzgk[2] = yk1 * sin(ik);
        double TempF[9] = {};
        matx(Rz, 3, 3, Rx, 3, 3, TempF);
        matx(TempF, 3, 3, xyzgk, 3, 1, xyzr);
        
        xyz[0] = xyzr[0];
        xyz[1] = xyzr[1];
        xyz[2] = xyzr[2];
    }

    //以下为计算卫星速度
    //计算偏近点角的时间导数
    double dotEk = n / (1 - satellite.e * cos(Ek));

    //计算真近点角的时间导数
    double dotvk = sqrt((1 + satellite.e) / (1 - satellite.e)) * cos(vk / 2) * cos(vk / 2) / cos(Ek / 2) / cos(Ek / 2) * dotEk;

    //计算升交角距的时间导数
    double dotuk = dotvk * (1 + 2 * satellite.cus * cos(2 * Phik) - 2 * satellite.cuc * sin(2 * Phik));

    //计算向径的时间导数
    double dotrk = dotEk * A * satellite.e * sin(Ek) + 2 * dotvk * (satellite.crs * cos(2 * Phik) - satellite.crc * sin(2 * Phik));

    //计算轨道倾角的时间导数
    double dotik = satellite.idot + 2 * dotvk * (satellite.cis * cos(2 * Phik) - satellite.cic * sin(2 * Phik));

    //计算卫星轨道平面位置的时间导数
    double dotxk = cos(uk) * dotrk - rk * sin(uk) * dotuk;
    double dotyk = sin(uk) * dotrk + rk * cos(uk) * dotuk;

    //计算卫星在地心坐标系下的速度并输出到结果数组
    double dotR[12] = { cos(Omegak),-sin(Omegak) * cos(ik),-xk1 * sin(Omegak) - yk1 * cos(Omegak) * cos(ik), yk1 * sin(Omegak) * sin(ik),
                        sin(Omegak), cos(Omegak) * cos(ik), xk1 * cos(Omegak) - yk1 * sin(Omegak) * cos(ik),-yk1 * cos(Omegak) * sin(ik),
                        0          , sin(ik)              , 0                                            , yk * cos(ik) };
    double tx[4] = { dotxk,dotyk,satellite.OMGD - OMGE_BDS,dotik };
    double dotxyzr[3] = {};
    matx(dotR, 3, 4, tx, 4, 1, dotxyzr);
    if (name > 5 && name < 59) {
        dotxyz[0] = dotxyzr[0], dotxyz[1] = dotxyzr[1], dotxyz[2] = dotxyzr[2];
    }
    else {
        double Ometk = OMGE_BDS * tk, f = -5.0 / 180 * pi;
        double dotR1[15] = {};
        double dotxyzrr[3] = {};
        double tx[5] = { dotxk,dotyk,satellite.OMGD,dotik,OMGE_BDS };
        dotR1[0] = cos(Ometk) * cos(Omegak1) + sin(Ometk) * cos(f) * sin(Omegak1);
        dotR1[1] = -cos(Ometk) * cos(ik) * sin(Omegak1) + sin(Ometk) * cos(f) * cos(ik) * cos(Omegak1) + sin(Ometk) * sin(f) * sin(ik);
        dotR1[2] = -xk1 * cos(Ometk) * sin(Omegak1) - yk1 * cos(ik) * cos(Ometk) * cos(Omegak1) + sin(Ometk) * cos(f) * xk1 * cos(Omegak1) - yk1 * cos(ik) * sin(Omegak1) * sin(Ometk) * cos(f);
        dotR1[3] = yk1 * sin(ik) * sin(Omegak1) * cos(Ometk) - yk1 * sin(Ometk) * cos(f) * sin(ik) * cos(Omegak1) + sin(Ometk) * sin(f) * yk1 * cos(ik);
        dotR1[4] = -sin(Ometk) * (xk1 * cos(Omegak1) - yk1 * cos(ik) * sin(Omegak1)) + cos(Ometk) * cos(f) * (xk1 * sin(Omegak1) + yk1 * cos(ik) * cos(Omegak1)) + cos(Ometk) * sin(f) * yk1 * sin(ik);
        
        dotR1[5] = -sin(Ometk) * cos(Omegak1) + cos(Ometk) * cos(f) * sin(Omegak1);
        dotR1[6] = sin(Ometk) * cos(ik) * sin(Omegak1) + cos(Ometk) * cos(f) * cos(ik) * cos(Omegak1) + cos(Ometk) * sin(f) * sin(ik);
        dotR1[7] = sin(Ometk) * xk1 * sin(Omegak1) + yk1 * sin(Ometk) * cos(ik) * cos(Omegak1) + cos(Ometk) * cos(f) * xk1 * cos(Omegak1) - yk1 * cos(Ometk) * cos(f) * cos(ik) * sin(Omegak1);
        dotR1[8] = -yk1 * sin(ik) * sin(Omegak1) * sin(Ometk) - cos(Ometk) * cos(f) * yk1 * sin(ik) * cos(Omegak1) + cos(Ometk) * sin(f) * yk1 * cos(ik);
        dotR1[9] = -cos(Ometk) * (xk1 * cos(Omegak1) - yk1 * cos(ik) * sin(Omegak1)) - sin(Ometk) * cos(f) * (xk1 * sin(Omegak1) + yk1 * cos(ik) * cos(Omegak1)) - sin(Ometk) * sin(f) * yk1 * sin(ik);
        
        dotR1[10] = -sin(f) * sin(Omegak1);
        dotR1[11] = -sin(f) * cos(ik) * cos(Omegak1) + cos(f) * sin(ik);
        dotR1[12] = -xk1 * sin(f) * cos(Omegak1) + yk1 * cos(ik) * sin(Omegak1) * sin(f);
        dotR1[13] = sin(f) * yk1 * sin(ik) * cos(Omegak1) + cos(f) * yk1 * cos(ik);
        dotR1[14] = 0;
        
        matx(dotR1, 3, 5, tx, 5, 1, dotxyzrr);
        dotxyz[0] = dotxyzrr[0], dotxyz[1] = dotxyzrr[1], dotxyz[2] = dotxyzrr[2];
    }
    
    //计算卫星钟差、钟速并输出到结果数组
    tdts = getdts(satellite, tobs) + Fc_BDS * satellite.e * sqrt(satellite.A) * sin(Ek) - satellite.tgd1;
    tdtss = satellite.af1 + 2 * satellite.af2 * (tobs.time + tobs.second - tdts - toc.time - toc.second) + Fc_BDS * satellite.e * sqrt(A) * cos(Ek) * dotEk;
}

//GPS卫星位置计算
void getsatelliteposition(eph_t satellite, char country, int name, double rho, double* xyz, double* dotxyz, gtime_t rt, double& tdts, double& tdtss) {
    if (satellite.statu == EPHNO)
    {
        xyz[0] = xyz[1] = xyz[2] = 0;
        dotxyz[0] = dotxyz[1] = dotxyz[2] = 0;
        tdts = tdtss = 0;
        return;
    }//星历不存在，不参与计算
    
    struct COMMONTIME comtime = {};

    gtime_t tobs = rt;//观测时间

    //计算卫星轨道长半轴
    double A = satellite.A;

    //计算平均运动角速度
    double n0 = sqrt(GM / A / A / A);//GM为地球引力常数

    //计算相对星历参考历元的时间

    double t = tobs.time - rho / clight - getdts(satellite, tobs) + satellite.tgd;
    double tk = t - satellite.toe.time;

    if (tk > 302400) tk = tk - 604800;
    else if (tk < -302400) tk = tk + 604800;
    else tk = tk;
    //对平均运动角速度进行改正Mk
    double n = satellite.deln + n0;
    //计算平进角点
    double Mk = satellite.M0 + n * tk;
    //计算偏近角点
    double Ek = Mk;
    double Ek1;
    while (1) {
        //Ek1=Ek-(Ek-satellite.e*sin(Ek)-Mk)/(1-satellite.e*cos(Ek));
        Ek1 = Mk + satellite.e * sin(Ek);
        if (abs(Ek1 - Ek) < 1e-13) break;
        Ek = Ek1;
    }
    Ek = Ek1;
    //计算真近点角vk
    double vk = atan2(sqrt(1 - satellite.e * satellite.e) * sin(Ek) / (1 - satellite.e * cos(Ek)), (cos(Ek) - satellite.e) / (1 - satellite.e * cos(Ek)));
    //计算升交点角距Phik
    double Phik = vk + satellite.omg;
    //计算二阶调和改正数
    double deltauk = satellite.cus * sin(2 * Phik) + satellite.cuc * cos(2 * Phik);
    double deltark = satellite.crs * sin(2 * Phik) + satellite.crc * cos(2 * Phik);
    double deltaik = satellite.cis * sin(2 * Phik) + satellite.cic * cos(2 * Phik);
    //计算改正的升交角距
    double uk = Phik + deltauk;
    //计算改正的向径
    double rk = A * (1 - satellite.e * cos(Ek)) + deltark;
    //计算改正的轨道倾角
    double ik = satellite.i0 + deltaik + satellite.idot * tk;
    //计算卫星在轨道平面上的位置
    double xk1 = rk * cos(uk);
    double yk1 = rk * sin(uk);
    //计算经过改正的升交点经度

    double Omegak = satellite.OMG0 + (satellite.OMGD - OMGE) * tk - OMGE * satellite.toes;
    //计算卫星在地心地固坐标系下位置
    double xk = xk1 * cos(Omegak) - yk1 * cos(ik) * sin(Omegak);
    double yk = xk1 * sin(Omegak) + yk1 * cos(ik) * cos(Omegak);
    double zk = yk1 * sin(ik);

    xyz[0] = xk;
    xyz[1] = yk;
    xyz[2] = zk;

    //以下为计算卫星速度
    //计算偏近点角的时间导数
    double dotEk = n / (1 - satellite.e*cos(Ek));

    //计算真近点角的时间导数
    double dotvk = sqrt((1+satellite.e)/(1-satellite.e))*cos(vk/2)*cos(vk/2)/cos(Ek/2)/cos(Ek/2)*dotEk;

    //计算升交角距的时间导数
    double dotuk = dotvk*(1+2*satellite.cus*cos(2*Phik)-2*satellite.cuc*sin(2*Phik));

    //计算向径的时间导数
    double dotrk = dotEk * A * satellite.e * sin(Ek) + 2*dotvk * (satellite.crs*cos(2*Phik)-satellite.crc*sin(2*Phik));

    //计算轨道倾角的时间导数
    double dotik = satellite.idot + 2 * dotvk * (satellite.cis*cos(2*Phik)-satellite.cic*sin(2*Phik));
    
    //计算卫星轨道平面位置的时间导数
    double dotxk = cos(uk) * dotrk - rk * sin(uk) * dotuk;
    double dotyk = sin(uk) * dotrk + rk * cos(uk) * dotuk;

    //计算卫星在地心坐标系下的速度并输出到结果数组
    double dotR[12] = { cos(Omegak),-sin(Omegak) * cos(ik),-xk1 * sin(Omegak) - yk1 * cos(Omegak) * cos(ik), yk1 * sin(Omegak) * sin(ik),
                        sin(Omegak), cos(Omegak) * cos(ik), xk1 * cos(Omegak) - yk1 * sin(Omegak) * cos(ik),-yk1 * cos(Omegak) * sin(ik),
                        0          , sin(ik)              , 0                                            , yk*cos(ik)                 };
    double tx[4] = { dotxk,dotyk,satellite.OMGD - OMGE,dotik };
    double dotxyzr[3] = {};
    matx(dotR, 3, 4, tx, 4, 1, dotxyzr);
    dotxyz[0]=dotxyzr[0], dotxyz[1]=dotxyzr[1], dotxyz[2]=dotxyzr[2];
    //计算卫星钟差、钟速并输出到结果数组
    tdts= getdts(satellite, tobs) + Fc * satellite.e * sqrt(satellite.A) * sin(Ek) - satellite.tgd;
    tdtss = satellite.af1 + 2*satellite.af2 * (tobs.time + tobs.second - tdts - satellite.toc.time - satellite.toc.second)+Fc*satellite.e*sqrt(A)*cos(Ek)*dotEk;
}


//单点定位用创建矩阵
double getR0(double* xyz0, double* xyz) {
    double R0;
    R0 = pow(xyz[0] - xyz0[0], 2) + pow(xyz[1] - xyz0[1], 2) + pow(xyz[2] - xyz0[2], 2);
    R0 = sqrt(R0);
    return R0;
}

double getl(double Xs, double X0, double R0) {
    return (Xs - X0) / R0;
}

double getm(double Ys, double Y0, double R0) {
    return (Ys - Y0) / R0;
}

double getn(double Zs, double Z0, double R0) {
    return (Zs - Z0) / R0;
}

void getmatrixL(double* Pi, double* R0, double* dts, double* dion, double* trp, int n, double* L) {
    int i;
    for (i = 0; i < n; i++) {
        L[i] = Pi[i] - R0[i] + clight * dts[i] - dion[i] - trp[i];
    }
}

void getmatrixB(double* l, double* m, double* n, int num, double* B) {
    int i;
    for (i = 0; i < num; i++) {
        B[4 * i + 0] = -l[i];
        B[4 * i + 1] = -m[i];
        B[4 * i + 2] = -n[i];
        B[4 * i + 3] = 1;
    }
}

void getmatrixP(double* var, int num, double* P) {
    for (int i = 0; i < num; i++)
        P[i * num + i] = 1.0 / var[i];
}

/*单点定位与多普勒测速结果结构体*/
typedef struct {
    char sysytem[20] = { "GPS" };//默认为GPS系统解算坐标
    double xyzt[4] = {};//定位结果
    double xyztspeed[4] = {};//测速结果
    double PDOP = 0;//三位点位衰减
    double TDOP = 0;//时间点位衰减
    double GDOP = 0;//几何点位衰减
    gtime_t tobs;
}SppResult;
//整合结果到结构体函数
SppResult get_SppResult(double Xr,double Yr,double Zr,double Dtr,double *dotxyzt,double pdot,double tdot,double gdot,gtime_t tobs) {
    SppResult Result;
    
    Result.xyzt[0] = Xr;Result.xyzt[1] = Yr;Result.xyzt[2] = Zr;Result.xyzt[3] = Dtr;

    Result.xyztspeed[0] = dotxyzt[0]; Result.xyztspeed[1] = dotxyzt[1]; 
    Result.xyztspeed[2] = dotxyzt[2]; Result.xyztspeed[3] = dotxyzt[3]/clight;
    //除以clight只需要除一次，如果函数外已经处理过了函数内就不用处理了
    
    Result.PDOP = pdot, Result.TDOP = tdot, Result.GDOP = gdot;

    Result.tobs = tobs;
    
    //本函数不改变定位系统名称，仍然保持默认为"GPS"
    
    return Result;
}

//单点定位与多普勒测速
/*参数解析*/
/*R：伪距观测值结构体*/
/*satllite：星历结构体数组*/
/*ion：电离层延迟参数数组*/
/*Result：计算结果结构体*/

/*GPS单系统单点定位*/
int SPPpos(GPSOBS R, eph_t* satllite, double* ion, SppResult& Result, int mode = 0) {
    int num = R.num; double USEFUL[36] = {}; int usenum = 0; float Cno[36] = {}; float psrstd[36] = {}; float dopp[36] = {};
    int sname[36] = {};
    for (int i = 0; i < num; i++)
        if (R.name[i] < 33&&satllite[R.name[i]].svh == 0&&satllite[R.name[i]].statu==EPHYES)
            USEFUL[usenum] = R.R0[i], sname[usenum] = R.name[i],Cno[usenum]=R.Cno[i],psrstd[usenum]=R.psrstd[i],dopp[usenum]=R.dopp[i], usenum++;
    num = usenum;

    //检查卫星数量
    if (num < 4) {
        printf("卫星数量太少，无法定位");
        return 0;
    }

    //初始化观测时间
    gtime_t tobs = R.rt;//观测时间
    //初始化单点定位、测速变量
    double dts[32] = {}, R0[32] = {}, l[32] = {}, m[32] = {}, n[32] = {}, xyz[3] = {}, Pi[32] = {}, xyz0[3] = { 100,100,100 },  dion[32] = {}, trp[32] = {}, var[32] = {};
    double dtss[32] = {}, dotxyz[3] = {}, w[32] = {};
    double PDOP, TDOP, GDOP, Xr, Yr, Zr, Dtr;
    int comcount = 0;
    while (1) {
        //参数获取
        for (int i = 0; i < num; i++) {
            //记录卫星编号并提取对应卫星的星历
            int name = sname[i];
            eph_t s = satllite[name];
            gtime_t st0 = s.toe;
            //记录name[i]卫星的伪距观测值
            Pi[i] = USEFUL[i]; double rho = Pi[i];
            //获取卫星位置与钟差
            double tdts,tdtss;
            getsatelliteposition(s, 'G', name, rho, xyz, dotxyz, tobs, tdts, tdtss);
            //获取name[i]号卫星的钟差钟速
            dts[i] = tdts; dtss[i] = tdtss;
            //地球自转改正
            double alpha = OMGE * (rho / clight + dts[i]);
            xyz[0] = cos(alpha) * xyz[0] + sin(alpha) * xyz[1];
            xyz[1] = cos(alpha) * xyz[1] - sin(alpha) * xyz[0];
            //计算几何距离R0，方向余弦
            R0[i] = getR0(xyz0, xyz);
            l[i] = getl(xyz[0], xyz0[0], R0[i]);
            m[i] = getm(xyz[1], xyz0[1], R0[i]);
            n[i] = getn(xyz[2], xyz0[2], R0[i]);
            //计算多普勒观测值
            double rorate = l[i] * dotxyz[0] + m[i] * dotxyz[1] + n[i] * dotxyz[2];
            w[i] = -0.19029367*dopp[i] - (rorate) + clight * dtss[i];//L1频率波长19.03cm,L2频率波长24.42cm,L5频率波长25.48cm
            //计算电离层,对流层延迟
            double azel[2];
            getazel(xyz, xyz0, azel);
            azel_GPS[name][0] = azel[0];
            azel_GPS[name][1] = azel[1];
            struct GPSTime tobsgps = {}; int week = 0;
            tobsgps.Second = time2gpst(tobs, week);
            tobsgps.Week = week;
            dion[i] = ionmodel(tobsgps, ion, xyz0, azel);
            trp[i] = tropmodel(xyz0, azel);
            //计算var(为构建P矩阵做准备)
            if (mode == 0)
                var[i] = 1;//单位阵格式
            //这个言之有理即可
            else if (mode == 1)
                var[i] = sqrt(7.2) + sqrt(abs(1.5 * dion[i])) + sqrt(0.9 / (sin(abs(azel[1])) + 0.1)) + sqrt(2.7) + sqrt(0.009 / sin(abs(azel[1])));
            //高度角模型（郑凯）
            else if (mode == 2)
                var[i] = 0.004 * 0.004 + 0.003 * 0.003 * cos(azel[1]) * cos(azel[1]);
            //高度角模型（教科书，仅适用于GPS）
            else if (mode == 3)
                var[i] = exp(psrstd[i]);
            //伪距观测值标准差(指数函数形式)(作者自定义的一种方法，慎重)
            else if (mode == 4)
                var[i] = 0.00224 * pow(10, -Cno[i] / 10);//信噪比模型1、载波L1参数为0.00224、L2参数为0.00077
            else if (mode == 5)
                var[i] = pow(10, -Cno[i] / 10);//不带参数的信噪比模型
            /*其他定权模型欢迎补充*/
        }

        //构建设计矩阵
        double B[32 * 4] = { 0 };
        getmatrixB(l, m, n, num, B);

        //组合观测值矩阵
        double L[32 * 1];
        getmatrixL(Pi, R0, dts, dion, trp, num, L);
        
        //权重经验矩阵
        double P[32 * 32] = { 0 };//权重经验矩阵
        getmatrixP(var, num, P);

        //计算矩阵Q
        double BT[32 * 4] = {};
        matT(B, num, 4, BT);
        double A[4 * 32] = {};
        matx(BT, 4, num, P, num, num, A);
        double Qr[4 * 4] = {};
        matx(A, 4, num, B, num, 4, Qr);
        double Q[4 * 4] = { 0 };
        inverseMatrix(Qr, 4, Q);

        //计算位置、速度的最小二乘估计值
        double X1[4 * 32] = {};
        matx(Q, 4, 4, BT, 4, num, X1);
        double X2[4 * 32] = {};
        matx(X1, 4, num, P, num, num, X2);
        double dxyzt[4] = {};
        matx(X2, 4, num, L, num, 1, dxyzt);//位置估计值

        double dotxyzt[4] = {};
        matx(X2, 4, num, w, num, 1, dotxyzt);//速度估计值
        
        //位置数值更新
        xyz0[0] = xyz0[0] + dxyzt[0];
        xyz0[1] = xyz0[1] + dxyzt[1];
        xyz0[2] = xyz0[2] + dxyzt[2];

        if ((abs(dxyzt[0]) < 1e-4 && abs(dxyzt[1]) < 1e-4 && abs(dxyzt[2]) < 1e-4) || comcount == 99) {
            //计算精度因子与接收机坐标、速度、钟差、钟速
            PDOP = sqrt(Q[0] + Q[5] + Q[10]);
            TDOP = sqrt(Q[15]);
            GDOP = sqrt(Q[0] + Q[5] + Q[10] + Q[15]);
            Xr = xyz0[0]; Yr = xyz0[1]; Zr = xyz0[2]; Dtr = dxyzt[3] / clight;
            Result=get_SppResult(Xr, Yr, Zr, Dtr, dotxyzt, PDOP, TDOP, GDOP,tobs);
            //printf("\n单点多普勒测速结果%lf %lf %lf %lf\n", dotxyzt[0], dotxyzt[1], dotxyzt[2], dotxyzt[3]/clight);
            break;
        }
        comcount++;
    }
    
    /*
    printf("根据卫星");
    for (int j = 0; j < num; j++)
        printf("G%02d ", sname[j]);
    printf("定位结果如下：\n");
    */
    return (comcount == 99) ? 999 : num;
}

/*北斗单系统单点定位*/
int SPPpos(GPSOBS R, eph_bds2* satllite, double* ion, SppResult& Result, int mode = 0) {
    
    //选星
    int num = R.num; double USEFUL[36] = {}; int usenum = 0; float Cno[36] = {}; float psrstd[36] = {}; float dopp[36] = {};
    int sname[36] = {};
    for (int i = 0; i < num; i++)
        if (satllite[R.name[i]].health == 0 && satllite[R.name[i]].statu == EPHYES)
            USEFUL[usenum] = R.R0[i], sname[usenum] = R.name[i], Cno[usenum] = R.Cno[i], psrstd[usenum] = R.psrstd[i], dopp[usenum] = R.dopp[i], usenum++;
    num = usenum;

    //检查卫星数量
    if (num < 4) {
        printf("卫星数量太少，无法定位");
        return 0;
    }

    //初始化观测时间
    gtime_t tobs = R.rt;//观测时间
    //初始化单点定位、测速变量
    double dts[32] = {}, R0[32] = {}, l[32] = {}, m[32] = {}, n[32] = {}, xyz[3] = {}, Pi[32] = {}, xyz0[3] = { 100,100,100 }, dion[32] = {}, trp[32] = {}, var[32] = {};
    double dtss[32] = {}, dotxyz[3] = {}, w[32] = {};
    double PDOP, TDOP, GDOP, Xr, Yr, Zr, Dtr;
    int computecount = 0;
    while (1) {
        //参数获取
        for (int i = 0; i < num; i++) {
            //记录卫星编号并提取对应卫星的星历
            int name = sname[i];
            eph_bds2 s = satllite[name];
            gtime_t st0 = gpst2time(s.week, s.toe);
            //记录name[i]卫星的伪距观测值
            Pi[i] = USEFUL[i]; double rho = Pi[i];
            //获取卫星位置与钟差
            double tdts, tdtss;
            BDSsatpos(s, 'C', name, rho, xyz, dotxyz, tobs, tdts, tdtss);
            //获取name[i]号卫星的钟差钟速
            dts[i] = tdts; dtss[i] = tdtss;
            //地球自转改正
            double alpha = OMGE_BDS * (rho / clight + dts[i]);
            xyz[0] = cos(alpha) * xyz[0] + sin(alpha) * xyz[1];
            xyz[1] = cos(alpha) * xyz[1] - sin(alpha) * xyz[0];
            //计算几何距离R0，方向余弦
            R0[i] = getR0(xyz0, xyz);
            l[i] = getl(xyz[0], xyz0[0], R0[i]);
            m[i] = getm(xyz[1], xyz0[1], R0[i]);
            n[i] = getn(xyz[2], xyz0[2], R0[i]);
            //计算多普勒观测值
            double rorate = l[i] * dotxyz[0] + m[i] * dotxyz[1] + n[i] * dotxyz[2];
            w[i] = -clight/1561.098e6 * dopp[i] - (rorate)+clight * dtss[i];//B1频率波长19.19cm,L2频率波长24.42cm,L5频率波长25.48cm
            //计算电离层,对流层延迟
            double azel[2];
            getazel(xyz, xyz0, azel);
            azel_BDS[name][0] = azel[0];
            azel_BDS[name][1] = azel[1];
            struct GPSTime tobsgps; int week;
            tobsgps.Second = time2gpst(tobs, week);
            tobsgps.Week = week;
            dion[i] = ionmodel_BDS(tobsgps, ion, xyz0, azel);
            trp[i] =  tropmodel(xyz0, azel);
            //计算var(为构建P矩阵做准备)
            if (mode == 0)
                var[i] = 1;//单位阵格式
            //这个言之有理即可
            else if (mode == 1)
                var[i] = sqrt(7.2) + sqrt(abs(1.5 * dion[i])) + sqrt(0.9 / (sin(abs(azel[1])) + 0.1)) + sqrt(2.7) + sqrt(0.009 / sin(abs(azel[1])));
            //高度角模型（郑凯）
            else if (mode == 2)
                var[i] = 0.004 * 0.004 + 0.003 * 0.003 * cos(azel[1]) * cos(azel[1]);
            //高度角模型（教科书，仅适用于GPS）
            else if (mode == 3)
                var[i] = exp(psrstd[i]);
            //伪距观测值标准差(指数函数形式)(作者自定义的一种方法，慎重)
            else if (mode == 4)
                var[i] = 0.00224 * pow(10, -Cno[i] / 10);//信噪比模型1、载波L1参数为0.00224、L2参数为0.00077
            else if (mode == 5)
                var[i] = pow(10, -Cno[i] / 10);//不带参数的信噪比模型
            
            /*其他定权模型欢迎补充*/
        }

        //构建设计矩阵
        double B[32 * 4] = { 0 };
        getmatrixB(l, m, n, num, B);

        //组合观测值矩阵
        double L[32 * 1];
        getmatrixL(Pi, R0, dts, dion, trp, num, L);

        //权重经验矩阵
        double P[32 * 32] = { 0 };//权重经验矩阵
        getmatrixP(var, num, P);

        //计算矩阵Q
        double BT[32 * 4] = {};
        matT(B, num, 4, BT);
        double A[4 * 32] = {};
        matx(BT, 4, num, P, num, num, A);
        double Qr[4 * 4] = {};
        matx(A, 4, num, B, num, 4, Qr);
        double Q[4 * 4] = { 0 };
        inverseMatrix(Qr, 4, Q);

        //计算位置、速度的最小二乘估计值
        double X1[4 * 32] = {};
        matx(Q, 4, 4, BT, 4, num, X1);
        double X2[4 * 32] = {};
        matx(X1, 4, num, P, num, num, X2);
        double dxyzt[4] = {};
        matx(X2, 4, num, L, num, 1, dxyzt);//位置估计值

        double dotxyzt[4] = {};
        matx(X2, 4, num, w, num, 1, dotxyzt);//速度估计值

        //位置数值更新
        xyz0[0] = xyz0[0] + dxyzt[0];
        xyz0[1] = xyz0[1] + dxyzt[1];
        xyz0[2] = xyz0[2] + dxyzt[2];

        if ((abs(dxyzt[0]) < 1e-6 && abs(dxyzt[1]) < 1e-6 && abs(dxyzt[2]) < 1e-6) || computecount == 99) {
            //计算精度因子与接收机坐标、速度、钟差、钟速
            PDOP = sqrt(Q[0] + Q[5] + Q[10]);
            TDOP = sqrt(Q[15]);
            GDOP = sqrt(Q[0] + Q[5] + Q[10] + Q[15]);
            Xr = xyz0[0]; Yr = xyz0[1]; Zr = xyz0[2]; Dtr = dxyzt[3] / clight;
            Result = get_SppResult(Xr, Yr, Zr, Dtr, dotxyzt, PDOP, TDOP, GDOP, tobs);
            //printf("\n单点多普勒测速结果%lf %lf %lf %lf\n", dotxyzt[0], dotxyzt[1], dotxyzt[2], dotxyzt[3]/clight);
            break;
        }
        computecount++;
    }
    return (computecount == 99) ? 999 : num;
}
//结果输出
void printResult(SppResult R,const char* mode="simple") {
    if ( strstr(mode,"simple"))
    {
        printf("%lld\n%lf\t%lf\t%lf\t%.6e\n", R.tobs.time, R.xyzt[0], R.xyzt[1], R.xyzt[2], R.xyzt[3]);
        printf("%lf\t%lf\t%lf\t%.6e\n", R.xyztspeed[0], R.xyztspeed[1], R.xyztspeed[2], R.xyztspeed[3]);
    }
    if (strstr(mode, "BLH"))
    {
        double blh[3] = {};
        xyztoblh(R.xyzt[0], R.xyzt[1], R.xyzt[2], blh);
        printf("%lld\n%lf\t%lf\t%lf\t%.6e\n", R.tobs.time, blh[0], blh[1], blh[2], R.xyzt[3]);
        printf("%lf\t%lf\t%lf\t%.6e\n", R.xyztspeed[0], R.xyztspeed[1], R.xyztspeed[2], R.xyztspeed[3]);

    }
    if (strstr(mode, "complex"))
    {
        double blh[3] = {};
        xyztoblh(R.xyzt[0], R.xyzt[1], R.xyzt[2], blh);
        printf("观测时间:对准卫星时间%d年%d月%d日%d:%d:%.2lf\n", time2epoch(R.tobs).Year,
            time2epoch(R.tobs).Month,
            time2epoch(R.tobs).Day,
            time2epoch(R.tobs).Hour,
            time2epoch(R.tobs).Minute,
            time2epoch(R.tobs).Second);
        printf("X=%.6lf, Y=%.6lf, Z=%.6lf, 接收机钟差为：%.6e s\n", R.xyzt[0], R.xyzt[1], R.xyzt[2], R.xyzt[3]);
        printf("vX=%.6lf, vY=%.6lf, vZ=%.6lf, 接收机钟速为：%.6e s\n", R.xyztspeed[0], R.xyztspeed[1], R.xyztspeed[2], R.xyztspeed[3]);
        printf("B=%lf,L=%lf,H=%lf\n", blh[0], blh[1], blh[2]);
        printf("三维点位精度衰减因子PDOP=%.6lf\n时间精度衰减因子TDOP=%.6lf\n几何精度衰减因子GDOP=%.6lf\n", R.PDOP, R.TDOP, R.GDOP);
    }
    if (strstr(mode, "usart")) {
        double blh[3] = {};
        xyztoblh(R.xyzt[0], R.xyzt[1], R.xyzt[2], blh);
        GPSTime tgps; int week;
        tgps.Second=time2gpst(R.tobs, week);
        tgps.Week = week;
        printf("%d %.2lf %lf %lf %lf %lf %lf %lf %d\n", tgps.Week, tgps.Second, R.xyzt[0], R.xyzt[1], R.xyzt[2], R.xyztspeed[0], R.xyztspeed[1], R.xyztspeed[2], 9);
    }
    if (strstr(mode, "CGCS2000")) {
        double blh[3] = {};
        xyztoblh_CGCS2000(R.xyzt[0], R.xyzt[1], R.xyzt[2], blh);
        printf("%lld\n%lf\t%lf\t%lf\t%.6e\n", R.tobs.time, blh[0], blh[1], blh[2], R.xyzt[3]);
        printf("%lf\t%lf\t%lf\t%.6e\n", R.xyztspeed[0], R.xyztspeed[1], R.xyztspeed[2], R.xyztspeed[3]);

    }
}
//结果保存
void fprintResult(FILE* fp, SppResult R) {
    double blh[3] = {};
    xyztoblh(R.xyzt[0], R.xyzt[1], R.xyzt[2], blh);
    fprintf(fp, "%s\t", R.sysytem);
    fprintf(fp,"%lld\t %lf\t %lf\t %lf\t %.6e\t ", R.tobs.time, R.xyzt[0], R.xyzt[1], R.xyzt[2], R.xyzt[3]);
    fprintf(fp, "%lf\t %lf\t %lf\t", blh[0], blh[1], blh[2]);
    fprintf(fp,"%lf\t %lf\t %lf\t %.6e\n", R.xyztspeed[0], R.xyztspeed[1], R.xyztspeed[2], R.xyztspeed[3]);
}
void fprintResult(FILE* fp, SppResult R,const char *temp) {
    double blh[3] = {};
    xyztoblh(R.xyzt[0], R.xyzt[1], R.xyzt[2], blh);
    fprintf(fp, "%lld\t %lf\t %lf\t %lf\t %.6e\n", R.tobs.time, blh[0], blh[1], blh[2], R.xyzt[3]);
}

/*结构体，有向二进制数组*/
typedef struct {
    unsigned char buff[8192] = {};
    int bufflen = 0;
}UARTbuff;


/**辅助函数：WGS-84坐标转GCJ02坐标部分**/

/*参数定义*/
#define PI 3.14159265358979323846
#define ee 0.00669342162296594323
#define aa 6378245.0

/*投影部分*/
double transform_lat(double lng, double lat)
{
    double ret = -100.0 + 2.0 * lng + 3.0 * lat + 0.2 * lat * lat + 0.1 * lng * lat + 0.2 * sqrt(abs(lng));
    ret = ret + (20.0 * sin(6.0 * lng * PI) + 20.0 * sin(2.0 * lng * PI)) * 2.0 / 3.0;
    ret = ret + (20.0 * sin(lat * PI) + 40.0 * sin(lat / 3.0 * PI)) * 2.0 / 3.0;
    ret = ret + (160.0 * sin(lat / 12.0 * PI) + 320 * sin(lat * PI / 30.0)) * 2.0 / 3.0;
    return ret;
}
double transform_lng(double lng, double lat)
{
    double ret = 300.0 + lng + 2.0 * lat + 0.1 * lng * lng + 0.1 * lng * lat + 0.1 * sqrt(abs(lng));
    ret = ret + (20.0 * sin(6.0 * lng * PI) + 20.0 * sin(2.0 * lng * PI)) * 2.0 / 3.0;
    ret = ret + (20.0 * sin(lng * PI) + 40.0 * sin(lng / 3.0 * PI)) * 2.0 / 3.0;
    ret = ret + (150.0 * sin(lng / 12.0 * PI) + 300.0 * sin(lng / 30.0 * PI)) * 2.0 / 3.0;
    return ret;
}
/*坐标转换部分，输入输出顺序：经度、纬度*/
void wgs84_to_gcj02(double lng, double lat, double* temp)
{
    double dlat = transform_lat(lng - 105.0, lat - 35.0);
    double dlng = transform_lng(lng - 105.0, lat - 35.0);
    double radlat = lat / 180.0 * PI;
    double magic = sin(radlat);
    magic = 1 - ee * magic * magic;
    double sqrtmagic = sqrt(magic);
    dlat = (dlat * 180.0) / ((aa * (1 - ee)) / (magic * sqrtmagic) * PI);
    dlng = (dlng * 180.0) / (aa / sqrtmagic * cos(radlat) * PI);
    temp[0] = lng + dlng;
    temp[1] = lat + dlat;
}

//GCJ02转BD09
void GCJ02ToBD09(double lon_GCJ, double lat_GCJ, double* BD) {
    double baiduFactor = (PI * 3000.0) / 180.0;
    double z = sqrt(lon_GCJ * lon_GCJ + lat_GCJ * lat_GCJ) + 0.00002 * sin(lat_GCJ * baiduFactor);
    double theta = atan2(lat_GCJ, lon_GCJ) + 0.000003 * cos(lon_GCJ * baiduFactor);
    BD[0] = z * cos(theta) + 0.0065;
    BD[1] = z * sin(theta) + 0.006;
}
//判断是否位于GCJ02坐标系统范围
int isInChinaBbox(double lon, double lat) {
    return lon >= 72.004 && lon <= 137.8347 && lat >= 0.8293 && lat <= 55.8271;
}


/*补充从RINEX格式文件中读取北斗星历的函数*/

void read_BDS_EPH_RINEX(const char* filname, eph_bds2* BDS, double ion[24][8]) {
    FILE* fp = fopen(filname, "r");
    char fullstr[200] = {};
    while (!feof(fp)) {
        fgets(fullstr, 200, fp);
        double tion[4] = {}; char time_char = '\0';
        if (strstr(fullstr, "BDSA") && strstr(fullstr, "IONOSPHERIC CORR"))
        {
            charreplace(fullstr, 'D', 'E');
            sscanf(fullstr + 4, "%lf %lf %lf %lf %c", &tion[0], &tion[1], &tion[2], &tion[3], &time_char);
            ion[time_char - 'a'][0] = tion[0], ion[time_char - 'a'][1] = tion[1];
            ion[time_char - 'a'][2] = tion[2], ion[time_char - 'a'][3] = tion[3];
        }

        if (strstr(fullstr, "BDSB") && strstr(fullstr, "IONOSPHERIC CORR"))
        {
            charreplace(fullstr, 'D', 'E');
            sscanf(fullstr + 4, "%lf %lf %lf %lf %c", &tion[0], &tion[1], &tion[2], &tion[3], &time_char);
            ion[time_char - 'a'][4] = tion[0], ion[time_char - 'a'][5] = tion[1];
            ion[time_char - 'a'][6] = tion[2], ion[time_char - 'a'][7] = tion[3];
        }

        if (strstr(fullstr, "END OF HEADER"))
            break;
    }
    //找到星历起始位置
    while (!feof(fp)) {
        fgets(fullstr, 200, fp);
        int name = 0; COMMONTIME ct = {};
        //读取第一行
        if (strstr(fullstr, "C"))
            sscanf(fullstr, "C%d", &name);
        else
            sscanf(fullstr, "%d", &name);
        charreplace(fullstr, 'D', 'E');//替换字符串
        sscanf(fullstr + 3, "%d %d %d %d %d %lf %lf %lf %lf", &ct.Year, &ct.Month, &ct.Day, &ct.Hour, &ct.Minute, &ct.Second,
            &BDS[name].af0, &BDS[name].af1, &BDS[name].af2);//第一行读取结束
        if (ct.Year < 100)
            ct.Year += 2000;//双数记录归2000
            
        //读取第二行
        fgets(fullstr, 200, fp);
        charreplace(fullstr, 'D', 'E');
        double t = 0.0;
        sscanf(fullstr, "%lf %lf %lf %lf", &t, &BDS[name].crs, &BDS[name].deltN, &BDS[name].M0);
        BDS[name].AODE1 = int(t);
        //读取第三行
        fgets(fullstr, 200, fp);
        charreplace(fullstr, 'D', 'E');
        sscanf(fullstr, "%lf %lf %lf %lf", &BDS[name].cuc, &BDS[name].e, &BDS[name].cus, &BDS[name].A);
        BDS[name].A = BDS[name].A * BDS[name].A;//平方根转原值
        //读取第四行
        fgets(fullstr, 200, fp);
        charreplace(fullstr, 'D', 'E');
        sscanf(fullstr, "%lf %lf %lf %lf", &BDS[name].toe, &BDS[name].cic, &BDS[name].OMG0, &BDS[name].cis);
        //读取第五行
        fgets(fullstr, 200, fp);
        charreplace(fullstr, 'D', 'E');
        sscanf(fullstr, "%lf %lf %lf %lf", &BDS[name].I0, &BDS[name].crc, &BDS[name].omg, &BDS[name].OMGD);
        //读取第六行
        fgets(fullstr, 200, fp); double tweek = 0.0;
        charreplace(fullstr, 'D', 'E');
        sscanf(fullstr, "%lf %lf %lf %lf", &BDS[name].idot, &t, &tweek, &t);
        BDS[name].week = tweek + 1356;//BDS周转GPS周
        //读取第七行
        fgets(fullstr, 200, fp);
        charreplace(fullstr, 'D', 'E');
        sscanf(fullstr, "%lf %lf %lf %lf", &BDS[name].URA, &t, &BDS[name].tgd1, &BDS[name].tgd2);
        BDS[name].health = unsigned long(t);
        //读取第八行
        fgets(fullstr, 200, fp);//开始读取第二行
        charreplace(fullstr, 'D', 'E');
        sscanf(fullstr, "%lf %lf %lf %lf", &t, &BDS[name].AODC, &t, &t);
        
        //余量数据更正与补全
        BDS[name].statu = EPHYES; int tempw = 0;
        BDS[name].toc = time2gpst(com2unixtime(ct), tempw);
        BDS[name].prn = name; BDS[name].Zweek = BDS[name].week;
        BDS[name].AODE2 = BDS[name].AODE1;
        BDS[name].URA = BDS[name].URA * BDS[name].URA;
    }
    fclose(fp);
}

/*补充串口缓冲区结构体*/
typedef struct {
    unsigned char buff[8192] = {};
    int bufflen = 0;
}USARTbuff;

/*补充:简化的Hopfiled模型对流层延迟解算函数*/
double Hopfiled_Trop(double* azel) {
    return 2.47 / (sin(azel[1]) + 0.0121);
}