#include<iostream>
#include"GNSStime.h"
#define MAXSIZE 1024*1024//�������1M��С�Ĺ۲��ļ�
#include <fstream>
#include<time.h>
#include<string.h>
#define Frame_IN 0
#define Frame_OUT 1//�����ļ�ָ��֡����
#define EPHYES 1
#define EPHNO 0
#define POLYCRC32 0xEDB88320u//CRCУ����
using namespace std;

unsigned short bit2ushort(unsigned char* p) {
    unsigned short r;
    memcpy(&r, p, 2);
    return r;
}
long bit2long(unsigned char* p) {
    long r;
    memcpy(&r, p, 4);
    return r;
}
double bit2double(unsigned char* p) {
    double r;
    memcpy(&r, p, 8);
    return r;
}
float bit2float(unsigned char* p) {
    float r;
    memcpy(&r, p, 4);
    return r;
}
unsigned long bit2ulong(unsigned char* p) {
    unsigned long r;
    memcpy(&r, p, 4);
    return r;
}

bool bit2bool(unsigned char* p) {
    bool r;
    memcpy(&r, p, 4);
    return r;
}

unsigned int bit2uint(unsigned char* p) {
    unsigned int r;
    memcpy(&r, p, 4);
    return r;
}


/*�ṹ�壺��Ϣ����*/
/*��������Ϣ�������еĿ�ʼ��������������Ϣ���ͣ�ID������Ϣ���ȣ�crcУ����*/
typedef struct {
    int start;
    int end;
    unsigned short ID;
    unsigned short Len;
    unsigned int crc;//CRCУ��
}breport;

/*�ṹ�壺GPS��������*/
/*������������������ָʾ����������*/
typedef struct {
    int statu = EPHNO;
    double af0, af1, af2;//�����Ӳ��������
    double tgd;//���ǵ�·Ⱥ��ʱ����
    bool AS;//AS��ʶ
    int svh;//���ǽ���״��(svh)
    int week; //GPS��
    gtime_t toe, toc; //���������Ĳο�ʱ�̣�������ʱ��
    double A, e; //���������(A)��ƫ����(e)
    double i0; //�����ǣ�i0)
    double OMG0; //�����㾭��(0MG0)
    double omg; //���ص�Ǿࣨomg��
    double M0; //ƽ����ǣ�M0��
    double deln; //ƽ�����ٶ�(deln)
    double OMGD; //���������ʣ�OMGd��
    double idot; //�����Ǳ仯�ʣ�idot��
    double crc, crs; //���ľ���㶯������(crc,crs)
    double cuc, cus; //�����Ǿ���㶯������(cuc,cus)
    double cic, cis; //��ǵ��㶯�����cic��cis��
    double toes; //������(toes)
    double N;//������ƽ�����ٶ�
}eph_t;

/*�ṹ�壺����������������*/
/*������������������ָʾ����������*/
typedef struct {
    int statu = EPHNO;//��ʼ��Ϊ����������������
    int prn;//����PRN��ţ�BDS 1-63��
    int health;//���ǽ���״̬,0=healthey
    unsigned char sattype;//�������(GE0/MEO/IGSO)
    unsigned char sismai;//�ռ��źż�⾫��
    unsigned short IODE, IODC;//������ʱ����������
    int week, Zweek;//��GPSʱΪ׼��������ע����ԭʼ���ݵ���������
    double tow;//��֡1��ʱ���ʶ
    double toe;//�����ο�ʱ�̣�����GPS�룩
    double DeltaA;//�ο�ʱ�̹������������ڲο�ֵ��ƫ��
    double dDeltaA;//������仯��
    double DeltaN;//�ο�ʱ������ƽ�����ٶ������ֵ֮��
    double dDeltaN;//�ο�ʱ������ƽ�����ٶ������ֵ֮��ı仯��
    double M0;//ƽ�����
    double e;//ƫ����
    double Omg;//���ص����
    double cuc, cus;//γ�ȷ����㶯����
    double crc, crs;//����뾶�㶯����
    double cic, cis;//�������㶯����
    double I0;//������
    double Idot;//�����Ǳ仯��
    double OMG0;//������ྭ
    double OMGD;//������ྭ�仯��
    double toc;//�ο�ʱ�̵�������ʱ��
    double tgdb1cp;//B1C��Ƶ����ʱ�Ӳ�
    double tgdB2ap;//B2A��Ƶ����ʱ�Ӳ�
    double ISCB2ad, ISCb1cd;//B2A/B1C���ݷ��������B2A/B1C��Ƶ������ʱ�������
    double af0, af1, af2;//�����Ӳ����
    int iTop;//����Ԥ�������ʱ��
    unsigned int FreqType;//�ź����ͣ�B1C/B2A��
}eph_bds3;

/*�ṹ�壺����������������*/
/*������������������ָʾ����������*/
typedef struct {
    int statu = EPHNO;//��ʼ��Ϊ����������������
    int prn;//����PRN��BDS1-63
    double tow;//��֡ 1 ��ʱ���ʶ������ GPS ʱ�䣩��s
    unsigned long health;//���ǽ���ָʾ��0=healthey
    unsigned long AODE1, AODE2;//������������
    unsigned long week, Zweek;//�����ܼ���������GPSʱΪ׼
    double toe;//ע�⣺����ʱ��������
    double A;//���������
    double deltN;//ƽ�����ٶȸ���ֵ
    double M0;//�ο�ʱ��ƽ�����
    double e;//ƫ����
    double omg;//���ص���ǣ��Ǿࣩ
    double cuc, cus, crc, crs, cic, cis;//γ�ȡ��뾶�͹����Ǹ�����
    double I0;//�ο�ʱ�̹�����
    double idot;//�����Ǳ仯��
    double OMG0;//������ྭ
    double OMGD;//������ྭ�仯��
    double AODC;//ʱ����������
    double toc;//�ο�ʱ�̵�������ʱ�䣬GPS��Ϊ׼
    double tgd1, tgd2;//Ⱥ�ӳ٣�B1��B2��
    double af0, af1, af2;//�Ӳ����
    double N;//������ƽ�����ٶ�
    double URA;//�û����뾫��
}eph_bds2;

void charreplace(char* str, char c, char b) {
    for (int i = 0; i < strlen(str); i++)
        if (str[i] == c)
            str[i] = b;
}

void eph2file(eph_t eph, FILE* fs, int prn) {
    COMMONTIME comtoc = time2epoch(eph.toc);
    char str[100] = {};
    fprintf(fs, "%2d%3d%3d%3d%3d%3d%5.1lf", prn, comtoc.Year % 100, comtoc.Month, comtoc.Day, comtoc.Hour, comtoc.Minute, comtoc.Second);
    sprintf(str, "%19.12E%19.12E%19.12E\n", eph.af0, eph.af1, eph.af2); charreplace(str, 'E', 'D'); fprintf(fs, str);//��һ��
    sprintf(str, "   %19.12E%19.12E%19.12E%19.12E\n", 48.0, eph.crs, eph.deln, eph.M0); charreplace(str, 'E', 'D'); fprintf(fs, str);
    sprintf(str, "   %19.12E%19.12E%19.12E%19.12E\n", eph.cuc, eph.e, eph.cus, sqrt(eph.A)); charreplace(str, 'E', 'D'); fprintf(fs, str);
    sprintf(str, "   %19.12E%19.12E%19.12E%19.12E\n", eph.toes, eph.cic, eph.OMG0, eph.cis); charreplace(str, 'E', 'D'); fprintf(fs, str);
    sprintf(str, "   %19.12E%19.12E%19.12E%19.12E\n", eph.i0, eph.crc, eph.omg, eph.OMGD); charreplace(str, 'E', 'D'); fprintf(fs, str);
    sprintf(str, "   %19.12E%19.12E%19.12E%19.12E\n", eph.idot, 0.0, double(eph.week), 1.0); charreplace(str, 'E', 'D'); fprintf(fs, str);
    sprintf(str, "   %19.12E%19.12E%19.12E%19.12E\n", 2.0, double(eph.svh), eph.tgd, 48.0); charreplace(str, 'E', 'D'); fprintf(fs, str);
    sprintf(str, "   %19.12E%19.12E%19.12E%19.12E\n", 0.0, 0.0, 0.0, 0.0); charreplace(str, 'E', 'D'); fprintf(fs, str);
}


GPSTime breport2GPStime(breport r, unsigned char* data) {
    GPSTime result = {};
    if (r.ID == 47 || r.ID == 43 || r.ID == 7 || r.ID == 631 || r.ID==1047) {
        unsigned char oweek[2] = { data[r.start + 14],data[r.start + 15] };
        unsigned char osec[4] = { data[r.start + 16],data[r.start + 17],data[r.start + 18],data[r.start + 19] };
        result.Week = bit2ushort(oweek);
        result.Second = double(bit2long(osec)) / 1000.0;
    }
    if (r.ID == 8) {
        unsigned char oweek[2] = { data[r.start + 10],data[r.start + 11] };
        unsigned char osec[4] = { data[r.start + 12],data[r.start + 13],data[r.start + 14],data[r.start + 15] };
        result.Week = bit2ushort(oweek);
        result.Second = double(bit2long(osec)) / 1000.0;
    }
    return result;
}

//CRCУ�麯��
unsigned int crc32(const unsigned char* buff, int len) {
    int i, j;
    unsigned int crc = 0;
    for (i = 0; i < len; i++) {
        crc ^= buff[i];
        for (j = 0; j < 8; j++) {
            if (crc & 1) crc = (crc >> 1) ^ POLYCRC32; else crc >>= 1;
        }
    }
    return crc;
}

//���CRCУ����
int getcrc(unsigned char* fullodata, breport r) {
    return crc32(fullodata + r.start, r.end - r.start - 3) == bit2uint(fullodata + r.end - 3);
}
//���ļ��л�ȡ��Ϣ����
int binaryfileread(breport* epoch, unsigned char* fullodata, FILE* fp) {
    unsigned char str[100];
    int epochnum = 0;
    int statu = Frame_OUT;//��ʼ��֡״̬Ϊ֡��
    int fcount = 0, count = 0;
    //�м�ؼ������洢
    unsigned char headerlen = 0, messagesID[2] = {}, messagesLen[2] = {};
    int flag = 0;//֡ͷ���ұ�־
    while (!feof(fp)) {
        fread(str, 1, 1, fp);
        fullodata[fcount] = str[0];
        fcount++;//ÿ�ζ�ȡһλ
    }
    //ͬ��֡ͷ
    for (int i = 0; i < fcount; i++)
    {
        if (i < fcount - 2 && fullodata[i] == 0xaa && fullodata[i + 1] == 0x44 && fullodata[i + 2] == 0x12)
            epoch[epochnum].start = i, statu = Frame_IN, headerlen = 28;
        if (i < fcount - 2 && fullodata[i] == 0xaa && fullodata[i + 1] == 0x44 && fullodata[i + 2] == 0xb5)
            epoch[epochnum].start = i, statu = Frame_IN, headerlen = 24;

        if (statu == Frame_IN && headerlen == 28) {
            count++;//��¼
            if (count == 5) messagesID[0] = fullodata[i];
            if (count == 6) messagesID[1] = fullodata[i];
            if (count == 9) messagesLen[0] = fullodata[i];
            if (count == 10) messagesLen[1] = fullodata[i];

            if (count > 28 && count == 28 + int(bit2ushort(messagesLen)) + 4) {
                //printf("%d\n",fcount); 
                epoch[epochnum].end = i;
                epoch[epochnum].ID = bit2ushort(messagesID);
                epoch[epochnum].Len = bit2ushort(messagesLen);
                epoch[epochnum].crc = getcrc(fullodata, epoch[epochnum]);
                epochnum++;
                statu = Frame_OUT; count = 0;
            }
        }
        if (statu == Frame_IN && headerlen == 24) {
            count++;//��¼
            if (count == 5) messagesID[0] = fullodata[i];
            if (count == 6) messagesID[1] = fullodata[i];
            if (count == 7) messagesLen[0] = fullodata[i];
            if (count == 8) messagesLen[1] = fullodata[i];

            if (count > 24 && count == 24 + int(bit2ushort(messagesLen)) + 4) {
                //printf("%d\n",fcount); 
                epoch[epochnum].end = i;
                epoch[epochnum].ID = bit2ushort(messagesID);
                epoch[epochnum].Len = bit2ushort(messagesLen);
                epoch[epochnum].crc = getcrc(fullodata, epoch[epochnum]);
                epochnum++;
                statu = Frame_OUT; count = 0;
            }
        }
    }
    return epochnum;
}
//��һ�����ȵĶ����������л�ȡ��Ϣ����
int getbinaryreport(breport* epoch, unsigned char* fullodata, int fulllen) {
    int epochnum = 0;
    int statu = Frame_OUT;//��ʼ��֡״̬Ϊ֡��
    int fcount = fulllen, count = 0;
    //�м�ؼ������洢
    unsigned char headerlen = 0, messagesID[2] = {}, messagesLen[2] = {};
    int flag = 0;//֡ͷ���ұ�־
    //ͬ��֡ͷ
    for (int i = 0; i < fcount; i++)
    {
        if (i < fcount - 2 && fullodata[i] == 0xaa && fullodata[i + 1] == 0x44 && fullodata[i + 2] == 0x12)
            epoch[epochnum].start = i, statu = Frame_IN, headerlen = 28;
        if (i < fcount - 2 && fullodata[i] == 0xaa && fullodata[i + 1] == 0x44 && fullodata[i + 2] == 0xb5)
            epoch[epochnum].start = i, statu = Frame_IN, headerlen = 24;

        if (statu == Frame_IN && headerlen == 28) {
            count++;//��¼
            if (count == 5) messagesID[0] = fullodata[i];
            if (count == 6) messagesID[1] = fullodata[i];
            if (count == 9) messagesLen[0] = fullodata[i];
            if (count == 10) messagesLen[1] = fullodata[i];

            if (count > 28 && count == 28 + int(bit2ushort(messagesLen)) + 4) {
                //printf("%d\n",fcount); 
                epoch[epochnum].end = i;
                epoch[epochnum].ID = bit2ushort(messagesID);
                epoch[epochnum].Len = bit2ushort(messagesLen);
                epoch[epochnum].crc = getcrc(fullodata, epoch[epochnum]);
                epochnum++;
                statu = Frame_OUT; count = 0;
            }
        }
        if (statu == Frame_IN && headerlen == 24) {
            count++;//��¼
            if (count == 5) messagesID[0] = fullodata[i];
            if (count == 6) messagesID[1] = fullodata[i];
            if (count == 7) messagesLen[0] = fullodata[i];
            if (count == 8) messagesLen[1] = fullodata[i];

            if (count > 24 && count == 24 + int(bit2ushort(messagesLen)) + 4) {
                //printf("%d\n",fcount); 
                epoch[epochnum].end = i;
                epoch[epochnum].ID = bit2ushort(messagesID);
                epoch[epochnum].Len = bit2ushort(messagesLen);
                epoch[epochnum].crc = getcrc(fullodata, epoch[epochnum]);
                epochnum++;
                statu = Frame_OUT; count = 0;
            }
        }
    }
    return epochnum;
}

//������������GPS�����ݶ�ȡ������MessageID 7
unsigned long getsat(eph_t* eph, unsigned char* fullodata, breport epoch) {
    int start = epoch.start, end = epoch.end;
    unsigned long prn = bit2ulong(fullodata + start + 28);
    if (prn > 35) return 0;
    eph[prn].svh = int(bit2ulong(fullodata + start + 28 + 12));
    eph[prn].week = int(bit2ulong(fullodata + start + 28 + 24));
    eph[prn].toes = bit2double(fullodata + start + 28 + 32);
    eph[prn].A = bit2double(fullodata + start + 28 + 40);
    eph[prn].deln = bit2double(fullodata + start + 28 + 48);
    eph[prn].M0 = bit2double(fullodata + start + 28 + 56);
    eph[prn].e = bit2double(fullodata + start + 28 + 64);
    eph[prn].omg = bit2double(fullodata + start + 28 + 72);
    eph[prn].cuc = bit2double(fullodata + start + 28 + 80);
    eph[prn].cus = bit2double(fullodata + start + 28 + 88);
    eph[prn].crc = bit2double(fullodata + start + 28 + 96);
    eph[prn].crs = bit2double(fullodata + start + 28 + 104);
    eph[prn].cic = bit2double(fullodata + start + 28 + 112);
    eph[prn].cis = bit2double(fullodata + start + 28 + 120);
    eph[prn].i0 = bit2double(fullodata + start + 28 + 128);
    eph[prn].idot = bit2double(fullodata + start + 28 + 136);
    eph[prn].OMG0 = bit2double(fullodata + start + 28 + 144);
    eph[prn].OMGD = bit2double(fullodata + start + 28 + 152);
    double etoc = bit2double(fullodata + start + 28 + 164);//������ʱ��
    eph[prn].tgd = bit2double(fullodata + start + 28 + 172);
    eph[prn].af0 = bit2double(fullodata + start + 28 + 180);
    eph[prn].af1 = bit2double(fullodata + start + 28 + 188);
    eph[prn].af2 = bit2double(fullodata + start + 28 + 196);
    eph[prn].AS = bit2ulong(fullodata + start + 28 + 204) ? true : false;
    eph[prn].N = bit2double(fullodata + start + 28 + 208);
    eph[prn].toe = gpst2time(eph[prn].week, eph[prn].toes);//���������Ĳο�ʱ��
    eph[prn].toc = gpst2time(eph[prn].week, etoc);
    return prn;//��������prn���
}

//������������������������ȡ����Message ID��3000
int getbds3eph(eph_bds3* eph, unsigned char* data, breport epoch)
{
    int start = epoch.start, end = epoch.end;
    unsigned char prnc = data[start + 28];
    int prn = int(prnc);//����PRN��
    eph[prn].prn = prn;
    eph[prn].health = int(data[start + 28 + 1]);
    eph[prn].sattype = data[start + 28 + 2];
    eph[prn].sismai = data[start + 28 + 3];
    eph[prn].IODE = bit2ushort(data + start + 28 + 4);
    eph[prn].IODC = bit2ushort(data + start + 28 + 6);
    eph[prn].week = int(bit2ushort(data + start + 28 + 8));
    eph[prn].Zweek = int(bit2ushort(data + start + 28 + 10));
    eph[prn].tow = bit2double(data + start + 28 + 12);
    eph[prn].toe = bit2double(data + start + 28 + 20);
    eph[prn].DeltaA = bit2double(data + start + 28 + 28);
    eph[prn].dDeltaA = bit2double(data + start + 28 + 36);
    eph[prn].DeltaN = bit2double(data + start + 28 + 44);
    eph[prn].dDeltaN = bit2double(data + start + 28 + 52);
    eph[prn].M0 = bit2double(data + start + 28 + 60);
    eph[prn].e = bit2double(data + start + 28 + 68);
    eph[prn].Omg = bit2double(data + start + 28 + 76);
    eph[prn].cuc = bit2double(data + start + 28 + 84);
    eph[prn].cus = bit2double(data + start + 28 + 92);
    eph[prn].crc = bit2double(data + start + 28 + 100);
    eph[prn].crs = bit2double(data + start + 28 + 108);
    eph[prn].cic = bit2double(data + start + 28 + 116);
    eph[prn].cis = bit2double(data + start + 28 + 124);
    eph[prn].I0 = bit2double(data + start + 28 + 132);
    eph[prn].Idot = bit2double(data + start + 28 + 140);
    eph[prn].OMG0 = bit2double(data + start + 28 + 148);
    eph[prn].OMGD = bit2double(data + start + 28 + 156);
    eph[prn].toc = bit2double(data + start + 28 + 164);
    eph[prn].tgdb1cp = bit2double(data + start + 28 + 172);
    eph[prn].tgdB2ap = bit2double(data + start + 28 + 180);
    eph[prn].ISCB2ad = bit2double(data + start + 28 + 188);
    eph[prn].ISCb1cd = bit2double(data + start + 28 + 196);
    eph[prn].af0 = bit2double(data + start + 28 + 204);
    eph[prn].af1 = bit2double(data + start + 28 + 212);
    eph[prn].af2 = bit2double(data + start + 28 + 220);
    eph[prn].iTop = bit2double(data + start + 28 + 228);
    eph[prn].FreqType = bit2uint(data + start + 28 + 244);
    return prn;
}
//������������ȡ����������������MessageID��1047
int getbds2eph(eph_bds2* eph, unsigned char* data, breport epoch)
{
    int start = epoch.start, end = epoch.end;
    unsigned long prnul = bit2ulong(data + start + 28);
    int prn = int(prnul);//����PRN��
    eph[prn].prn = prn;
    eph[prn].tow = bit2double(data + start + 28 + 4);
    eph[prn].health = bit2ulong(data + start + 28 + 12);
    eph[prn].AODE1 = bit2ulong(data + start + 28 + 16);
    eph[prn].AODE2 = bit2ulong(data + start + 28 + 20);
    eph[prn].week = bit2ulong(data + start + 28 + 24);
    eph[prn].Zweek = bit2ulong(data + start + 28 + 28);
    eph[prn].toe = bit2double(data + start + 28 + 32);
    eph[prn].A = bit2double(data + start + 28 + 40);
    eph[prn].deltN = bit2double(data + start + 28 + 48);
    eph[prn].M0 = bit2double(data + start + 28 + 56);
    eph[prn].e = bit2double(data + start + 28 + 64);
    eph[prn].omg = bit2double(data + start + 28 + 72);
    eph[prn].cuc = bit2double(data + start + 28 + 80);
    eph[prn].cus = bit2double(data + start + 28 + 88);
    eph[prn].crc = bit2double(data + start + 28 + 96);
    eph[prn].crs = bit2double(data + start + 28 + 104);
    eph[prn].cic = bit2double(data + start + 28 + 112);
    eph[prn].cis = bit2double(data + start + 28 + 120);
    eph[prn].I0 = bit2double(data + start + 28 + 128);
    eph[prn].idot = bit2double(data + start + 28 + 136);
    eph[prn].OMG0 = bit2double(data + start + 28 + 144);
    eph[prn].OMGD = bit2double(data + start + 28 + 152);
    eph[prn].AODC = bit2ulong(data + start + 28 + 160);
    eph[prn].toc = bit2double(data + start + 28 + 164);
    eph[prn].tgd1 = bit2double(data + start + 28 + 172);
    eph[prn].tgd2 = bit2double(data + start + 28 + 180);
    eph[prn].af0 = bit2double(data + start + 28 + 188);
    eph[prn].af1 = bit2double(data + start + 28 + 196);
    eph[prn].af2 = bit2double(data + start + 28 + 204);
    eph[prn].N = bit2double(data + start + 28 + 216);
    eph[prn].URA = bit2double(data + start + 28 + 224);
    return prn;
};
//������һ������
void bdseph2gpseph(eph_t* gps, eph_bds2* bds, int prn) {
    
    gps[prn].svh = int(bds[prn].health);
    gps[prn].week = int(bds[prn].week);
    gps[prn].toes = bds[prn].toe;
    gps[prn].A = bds[prn].A;
    gps[prn].deln = bds[prn].deltN;
    gps[prn].M0 = bds[prn].M0;
    gps[prn].e = bds[prn].e;
    gps[prn].omg = bds[prn].omg;
    gps[prn].cuc = bds[prn].cuc;
    gps[prn].cus = bds[prn].cus;
    gps[prn].crc = bds[prn].crc;
    gps[prn].crs = bds[prn].crs;
    gps[prn].cic = bds[prn].cic;
    gps[prn].cis = bds[prn].cis;
    gps[prn].i0 = bds[prn].I0;
    gps[prn].idot = bds[prn].idot;
    gps[prn].OMG0 = bds[prn].OMG0;
    gps[prn].OMGD = bds[prn].OMGD;
    double etoc = bds[prn].toc;//������ʱ��
    gps[prn].tgd = bds[prn].tgd1;
    gps[prn].af0 = bds[prn].af0;
    gps[prn].af1 = bds[prn].af1;
    gps[prn].af2 = bds[prn].af2;
    gps[prn].N = bds[prn].N;
    gps[prn].toe = gpst2time(gps[prn].week, gps[prn].toes);//���������Ĳο�ʱ��
    gps[prn].toc = gpst2time(gps[prn].week, etoc);
}
/*�ṹ�壺GPSϵͳ�Ĺ۲�ֵ*/
/*�������۲�ʱ��͹۲�ֵ*/
typedef struct {
    int num = 0;//�۲⵽���ǵ�����
    gtime_t rt;//�۲�ʱ��
    int name[36] = {};//��¼���Ǳ��
    double R0[36] = {};//α��۲�ֵ
    float psrstd[36] = {};//α���׼��
    double adr[36] = {};//�ز���λ
    float adrstd[36] = {};//�ز���λ��׼��
    float dopp[36] = {};//˲ʱ������Hz
    float  Cno[36] = {};//�����dB-Hz
    float loctime[36] = {};//��������ʱ��(������)
    unsigned long trackstatu[36] = {};//��������״̬
}GPSOBS;

int getobs(GPSOBS& R, unsigned char* fullodata, breport epoch) {
    int start = epoch.start, end = epoch.end, snum, name[35];
    double R0[35] = {};
    //�۲�ʱ��
    COMMONTIME comt = time2epoch(gpst2time(int(breport2GPStime(epoch, fullodata).Week), breport2GPStime(epoch, fullodata).Second));
    snum = bit2long(fullodata + start + 28);
    R.num = snum;
    GPSTime obswas = breport2GPStime(epoch, fullodata);
    R.rt = gpst2time(obswas.Week, obswas.Second);
    for (int j = 0; j < snum; j++)
    {
        //PRN��
        R.name[j] = int(bit2ushort(fullodata + start + 32 + j * 44));
        //α��۲�ֵ
        R.R0[j] = bit2double(fullodata + start + 36 + j * 44);
        R.psrstd[j] = bit2float(fullodata + start + 28 + 16 + j * 44);
        R.adr[j] = bit2double(fullodata + start + 28 + 20 + j * 44);
        R.adrstd[j] = bit2float(fullodata + start + 28 + 28 + j * 44);
        R.dopp[j] = bit2float(fullodata + start + 28 + 32 + j * 44);
        R.Cno[j] = bit2float(fullodata + start + 28 + 36 + j * 44);
        R.loctime[j] = bit2float(fullodata + start + 28 + 40 + j * 44);
    }
    return snum;
}

