#pragma execution_character_set("utf-8")//防中文乱码
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
    
    void RecesiveAreaInit(void);//串口接收区初始化
    void SendAreaInit(void);//串口发送区初始化
    void SerialSetupInit(void);//串口配置初始化
    void StartUSART(void);//打开串口
    void USART(void);//打开串口功能函数
    void timerEvent(QTimerEvent* e);//定时器事件（唯一，不可更名）
    void Refreshports(void);//可用串口编号更新
    void Refreshfitmodel(void);//定权模型更新

    void computingshowAreaInit(void);
    int RefreshUsedmessage(unsigned char* message);
    void computing(unsigned char* messsage, int messagex);

    void Serial_CMD_System(void);

    void mapshowInit(void);//高德地图显示区域初始化
    void mapmarker(double lng, double lat);//在高德地图上以WGS-84经纬度坐标插入点标记

    void paintEvent(QPaintEvent* e);

    void dateLCDInit(void);//LCD显示协调世界时初始化
    void RefreshLCDtime(void);//更新时间

    void File_solveInit(void);//文件解算区初始化
   
    void FilesolvepageInit(void);//文件结算界面初始化

    int File_SAT_POS(void);//基于文件解算卫星位置的计算函数

    void BINEARY_sat(const char* file, int mode = 1);//二进制文件解算
    void RINEX_GPS_sat(const char* SAT_filename, const char* OBS_filename, int mode = 1);//从RINEX格式的文件中进行GPS单系统定位
    void RINEX_BDS_sat(const char* SAT_filename, const char* OBS_filename, int mode = 1);//从RINEX格式的文件中进行BDS单系统定位

    void starthelper(void);//帮助系统初始化
    void HelperInit(void);//帮助系统子线程

    void btn_label_fontresize(void);//组件文字按比例更改
    

private:
    
    double hb = 1.0, wb = 1.0;//屏幕比例因子
    
    int BUFFSIZE = 8192;
    QPlainTextEdit* sendArea;
    QPlainTextEdit* receiveArea;
    QPlainTextEdit* computingshowArea;

    QPushButton* sendButton;
    QPushButton* startUSART;
    QPushButton* endUSART;

    QPushButton* Realtimemap;
    QPushButton* shutRealtimemap;
    

    QComboBox* portnum;//端口号
    QComboBox* botrate;//波特率
    QComboBox* datasize;//数据位
    QComboBox* checksize;//校验位
    QComboBox* stopsize;//停止位
    QComboBox* resMode;//接收模式
    QComboBox* sendMode;//发送模式

    QComboBox* Systemselect;//卫星系统选择模式
    QComboBox* Saveselect;//保存模式

    QComboBox* Fitmodel;//定权模型选择
    
    QSerialPort* serialPort;//串口对象
    QVector<QString>ports;//设备在用串口编号列表

    unsigned char buffer[8192] = {};//串口缓冲区二进制数组
    int slen = 0;//串口缓冲区数据长度
    
    QPushButton* realtimesolveButton;//实时流自解算按钮
    QPushButton* shutsolveButton;//实时流自解算停止按钮
    int realtimesolveflag = 0;//实时流自解算标志

    QWebEngineView* Gaodemapshow;//高德地图展示的web容器
    QWebChannel* Gaodemapchannel;//建立cpp<-html的连接，暂未使用
    QPlainTextEdit* Latedit;//高德地图单点标记纬度输入
    QPlainTextEdit* Lngedit;//高德地图单点标记经度输入
    QString htmlFilePath = QApplication::applicationDirPath() + "/mapsource/Baidumap.html";//地图文件路径
    
    
    QLCDNumber* dateshow;

    int portnum_TimeID = 0;//串口更新时间的计时器ID
    int map_TimeID = 0;//实时流地图标记点更新计时器ID
    int painter_TimeID = 0;//实时流卫星方位角更新计时器ID

    int Fitmodelnum = 0;//定权模型编号

    QPainter* azelShow;//绘图区对象

    QPlainTextEdit* OBSfilechoice;//观测值文件选择
    QPlainTextEdit* SATfilechoice;// 星历文件选择

    QComboBox* Fileselect;//文件格式选择
    QComboBox* File_system_select;//解算模式选择(卫星位置/测站位置)

    QWidget* Filesolvepage;//文件解算界面
    QPlainTextEdit* solveresultshow;
    QPushButton* replaybt;//文件解算测站位置轨迹回放按钮
    Stage children;//中间值解算子界面

    QWidget* Helpercontainer;//子线程：帮助程序
    QWebEngineView* Helper;//帮助文档显示

};
