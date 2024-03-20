#pragma execution_character_set("utf-8")//����������
#include <QtWidgets/QMainWindow>
#include<QTimer>
//#include<QDebug>

#include <QPlainTextEdit>
#include<QPushButton>
#include<QCombobox>
#include<QLabel>
#include<QDebug>
#include<QSerialPort>
#include<QSerialPortInfo>
#include<QmessageBox>
#include <QtWebEngineWidgets/QWebEngineView>
#include<QNetworkProxyFactory>
#include<QApplication>
#include<QPainter>
#include<QLCDnumber>
#include<QFile>
#include<QFileDialog>
#include<QHBoxLayout>
#include<QScreen>
#include<QWebEngineSettings>

#include"Stage.h"


class UBSerials : public QMainWindow
{
    Q_OBJECT

public:
    
    UBSerials(QWidget *parent = nullptr);
    ~UBSerials(); 
    
    void RecesiveAreaInit(void);//���ڽ�������ʼ��
    void SendAreaInit(void);//���ڷ�������ʼ��
    void SerialSetupInit(void);//�������ó�ʼ��
    void StartUSART(void);//�򿪴���
    void USART(void);//�򿪴��ڹ��ܺ���
    void timerEvent(QTimerEvent* e);//��ʱ���¼���Ψһ�����ɸ�����
    void Refreshports(void);//���ô��ڱ�Ÿ���
    void Refreshfitmodel(void);//��Ȩģ�͸���

    void computingshowAreaInit(void);
    int RefreshUsedmessage(unsigned char* message);
    void computing(unsigned char* messsage, int messagex);

    void Serial_CMD_System(void);

    void mapshowInit(void);//�ߵµ�ͼ��ʾ�����ʼ��
    void mapmarker(double lng, double lat);//�ڸߵµ�ͼ����WGS-84��γ������������

    void paintEvent(QPaintEvent* e);

    void dateLCDInit(void);//LCD��ʾЭ������ʱ��ʼ��
    void RefreshLCDtime(void);//����ʱ��

    void File_solveInit(void);//�ļ���������ʼ��
   
    void FilesolvepageInit(void);//�ļ���������ʼ��

    int File_SAT_POS(void);//�����ļ���������λ�õļ��㺯��

    void BINEARY_sat(const char* file, int mode = 1);//�������ļ�����
    void RINEX_GPS_sat(const char* SAT_filename, const char* OBS_filename, int mode = 1);//��RINEX��ʽ���ļ��н���GPS��ϵͳ��λ
    void RINEX_BDS_sat(const char* SAT_filename, const char* OBS_filename, int mode = 1);//��RINEX��ʽ���ļ��н���BDS��ϵͳ��λ

    void starthelper(void);//����ϵͳ��ʼ��
    void HelperInit(void);//����ϵͳ���߳�

    void btn_label_fontresize(void);//������ְ���������
    

private:
    
    double hb = 1.0, wb = 1.0;//��Ļ��������
    
    int BUFFSIZE = 8192;
    QPlainTextEdit* sendArea;
    QPlainTextEdit* receiveArea;
    QPlainTextEdit* computingshowArea;

    QPushButton* sendButton;
    QPushButton* startUSART;
    QPushButton* endUSART;

    QPushButton* Realtimemap;
    QPushButton* shutRealtimemap;
    

    QComboBox* portnum;//�˿ں�
    QComboBox* botrate;//������
    QComboBox* datasize;//����λ
    QComboBox* checksize;//У��λ
    QComboBox* stopsize;//ֹͣλ
    QComboBox* resMode;//����ģʽ
    QComboBox* sendMode;//����ģʽ

    QComboBox* Systemselect;//����ϵͳѡ��ģʽ
    QComboBox* Saveselect;//����ģʽ

    QComboBox* Fitmodel;//��Ȩģ��ѡ��
    
    QSerialPort* serialPort;//���ڶ���
    QVector<QString>ports;//�豸���ô��ڱ���б�

    unsigned char buffer[8192] = {};//���ڻ���������������
    int slen = 0;//���ڻ��������ݳ���
    
    QPushButton* realtimesolveButton;//ʵʱ���Խ��㰴ť
    QPushButton* shutsolveButton;//ʵʱ���Խ���ֹͣ��ť
    int realtimesolveflag = 0;//ʵʱ���Խ����־

    QWebEngineView* Gaodemapshow;//�ߵµ�ͼչʾ��web����
    QWebChannel* Gaodemapchannel;//����cpp<-html�����ӣ���δʹ��
    QPlainTextEdit* Latedit;//�ߵµ�ͼ������γ������
    QPlainTextEdit* Lngedit;//�ߵµ�ͼ�����Ǿ�������
    QString htmlFilePath = QApplication::applicationDirPath() + "/mapsource/Baidumap.html";//��ͼ�ļ�·��
    
    
    QLCDNumber* dateshow;

    int portnum_TimeID = 0;//���ڸ���ʱ��ļ�ʱ��ID
    int map_TimeID = 0;//ʵʱ����ͼ��ǵ���¼�ʱ��ID
    int painter_TimeID = 0;//ʵʱ�����Ƿ�λ�Ǹ��¼�ʱ��ID

    int Fitmodelnum = 0;//��Ȩģ�ͱ��

    QPainter* azelShow;//��ͼ������

    QPlainTextEdit* OBSfilechoice;//�۲�ֵ�ļ�ѡ��
    QPlainTextEdit* SATfilechoice;// �����ļ�ѡ��

    QComboBox* Fileselect;//�ļ���ʽѡ��
    QComboBox* File_system_select;//����ģʽѡ��(����λ��/��վλ��)

    QWidget* Filesolvepage;//�ļ��������
    QPlainTextEdit* solveresultshow;
    QPushButton* replaybt;//�ļ������վλ�ù켣�طŰ�ť
    Stage children;//�м�ֵ�����ӽ���

    QWidget* Helpercontainer;//���̣߳���������
    QWebEngineView* Helper;//�����ĵ���ʾ

};
