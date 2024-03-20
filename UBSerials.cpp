#include "UBSerials.h"
#include"satpos.h"
#pragma 全局变量
SppResult Result = {};//定位结果结构体
UARTbuff Left = {};//剩余数据区
UARTbuff buff = {};//缓冲区
unsigned char message[2 * 8192] = {};//数据区(上次缓冲区剩余＋本次缓冲区)
//定义接收机启动后的全局变量
eph_t eph[36] = {};//GPS星历
eph_bds2 bds[65] = {};//BDS星历
double ion[8] = {}; double ion_bds[8];//电离层延迟改正参数结构体
GPSOBS R = {};//观测值结构体
gtime_t tobsnow = {};//当前GPS时间
long bdt_utc = 0;//当前BDS与UTC改正数，经BDS星历播发

FILE* fp, *fo, *fs;//用于原始二进制文件保存的文件指针


#pragma 接收区初始化
void UBSerials::RecesiveAreaInit(void) {

    //接收区初始化
    receiveArea = new QPlainTextEdit(this);
    receiveArea->setFixedSize(800 * wb, 400 * hb);
    receiveArea->move(320*wb, 20*hb);
    receiveArea->setReadOnly(true);

    //清空接收区按钮及其槽函数
    QPushButton* clearResArea = new QPushButton("清空接收区", this);
    clearResArea->setFixedSize(150 * wb, 50 * hb);
    clearResArea->move(320*wb, 430*hb);

    connect(clearResArea, &QPushButton::clicked, [&]() {
        receiveArea->clear();
        });
    //接收区标签
    QLabel* resArealabel = new QLabel("串口数据接收区（原始数据交互界面）", this);
    resArealabel->setFixedSize(400 * wb, 50 * hb);
    resArealabel->move(520*wb, 430*hb);

}

#pragma 自解算结果显示区域初始化
void UBSerials::computingshowAreaInit(void) {
    //自解算结果显示区初始化
    computingshowArea = new QPlainTextEdit(this);
    computingshowArea->setFixedSize(400 * wb, 400 * hb);
    computingshowArea->move(200*wb, 730*hb);
    computingshowArea->setReadOnly(true);
    
    //自解算显示区标签
    QLabel* Systemselectlabel = new QLabel("导航系统", this);//系统选择框标签
    Systemselect = new QComboBox(this);//系统选择框
    QLabel* Saveselectlabel = new QLabel("文件保存", this);//解算结果保存模式选择标签
    Saveselect = new QComboBox(this);//解算结果保存模式选择框
    QLabel* Fitmodellabel = new QLabel("定权模型", this);//定权模型选择标签
    Fitmodel = new QComboBox(this);//定权模型选择框

    Systemselect->addItem("GPS");
    Systemselect->addItem("BDS");
    
    Saveselect->addItem("不保存");
    Saveselect->addItem("保存");

    Fitmodel->addItem("单位阵");
    Fitmodel->addItem("高度角1");
    Fitmodel->addItem("高度角2");
    Fitmodel->addItem("伪距标准差");
    Fitmodel->addItem("L1信噪比");
    Fitmodel->addItem("信噪比");

    Systemselectlabel->move(30*wb, 720*hb);
    Systemselectlabel->setFixedSize(150 * wb, 40 * hb);
    Systemselect->setFixedSize(150 * wb, 40 * hb);
    Systemselect->move(30*wb, 760*hb);

    Saveselectlabel->move(30*wb, 820*hb);
    Saveselectlabel->setFixedSize(150 * wb, 40 * hb);
    Saveselect->setFixedSize(150 * wb, 40 * hb);
    Saveselect->move(30*wb, 860*hb);

    Fitmodellabel->move(30*wb, 920*hb);
    Fitmodellabel->setFixedSize(150*wb, 40*hb);
    Fitmodel->setFixedSize(150 * wb, 40 * hb);
    Fitmodel->move(30*wb, 960*hb);

    //初始化自解算按钮
    realtimesolveButton = new QPushButton("开启自解算", this);
    realtimesolveButton->setFixedSize(150 * wb, 50 * hb);
    realtimesolveButton->move(30*wb, 1020*hb);
    realtimesolveButton->setDisabled(false);//一开始只能打开自解算

    connect(realtimesolveButton, &QPushButton::clicked, [&]() {
        if (serialPort==NULL||!serialPort->isOpen()) {
            QMessageBox::critical(this, "串口打开失败", "请确认串口是否正确连接");
        }
        else {
            this->realtimesolveflag = 1;//更改自解算功能标志
            shutsolveButton->setDisabled(false);//shut使能
            realtimesolveButton->setDisabled(true);//set关闭

            //二进制文件保存路径文件对话框
            if (Saveselect->currentText() == "保存") {
                QString currentpath = QDir::currentPath() + "/Example/";//当前路径
                QString dlgtitle = "选择二进制文件和结果文件保存路径";//文件对话框名称
                QString filter = "所有文件(*.*)";//文件显示配置
                QString folderName = QFileDialog::getExistingDirectory(this, dlgtitle, currentpath);

                QDateTime current_date_time = QDateTime::currentDateTime();
                QString date_time = current_date_time.toString("yyyyMMddhhmm");
                QString ofpath = folderName + "/" + date_time + "_OriginalOBS.txt";
                QString orpath = folderName + "/" + date_time + "_SolvedResult.txt";
                char t1[200] = {}, t2[200] = {};
                fp = fopen(ofpath.toLocal8Bit().data(), "wb");//原始数据保存
                fs = fopen(orpath.toLocal8Bit().data(), "w");
                fprintf(fs, "Sys GPST X Y Z dt B L H vX vY vZ vdt\n");
                Saveselect->setDisabled(true);
            }
            
            Serial_CMD_System();//一切准备就绪，写入串口命令
        }
        });
    
    //初始化自解算停止按钮
    shutsolveButton = new QPushButton("关闭自解算", this);
    shutsolveButton->setFixedSize(150 * wb, 50 * hb);
    shutsolveButton->move(30*wb, 1080*hb);
    shutsolveButton->setDisabled(true);//一开始无法关停自解算

    connect(shutsolveButton, &QPushButton::clicked, [&]() {
        this->realtimesolveflag = 0;//更改自解算功能标志
        realtimesolveButton->setDisabled(false);
        shutsolveButton->setDisabled(true);

        serialPort->write("unlog\r\n");

        if (Saveselect->currentText() == "保存") {
            fclose(fp);
            fclose(fs);
            Saveselect->setDisabled(false);
        }
        });

}

#pragma 串口发送区初始化
void UBSerials::SendAreaInit(void) {
    //发送区初始化
    sendArea = new QPlainTextEdit(this);
    sendArea->setFixedSize(800 * wb, 100 * hb);
    sendArea->move(320*wb, 500*hb);

    //发送按钮初始化及其槽函数
    sendButton = new QPushButton("发送", this);
    sendButton->setFixedSize(150 * wb, 50 * hb);
    sendButton->move(320*wb, 630*hb);
    sendButton->setDisabled(true);

    connect(sendButton, &QPushButton::clicked, [&]() {
        QString data = sendArea->toPlainText();
        data.append("\r\n");//自动添加命令所需的换行符和回车键
        if (sendMode->currentText() == "HEX") {
            QByteArray arr;
            for (int i = 0; i < data.size(); i++) {
                int num = data.mid(i, 2).toUInt(nullptr, 16);
                i++;
                arr.append(num);
            }
            serialPort->write(arr);
        }
        else {
            //serialPort->write(data.toLocal8Bit().data());
            serialPort->write(data.toUtf8().data());
        }
        });

    //清空发送区按钮及其槽函数
    QPushButton* clearsendArea = new QPushButton("清空发送区", this);
    clearsendArea->setFixedSize(150 * wb, 50 * hb);
    clearsendArea->move(480*wb, 630*hb);

    connect(clearsendArea, &QPushButton::clicked, [&]() {
        sendArea->clear();
        });
}

#pragma 串口配置初始化
void UBSerials::SerialSetupInit(void) {
    //新建combbox对象
    this->portnum = new QComboBox(this);
    this->botrate = new QComboBox(this);
    this->datasize = new QComboBox(this);
    this->checksize = new QComboBox(this);
    this->stopsize = new QComboBox(this);
    this->sendMode = new QComboBox(this);
    this->resMode = new QComboBox(this);


    //为每个combox添加选项
    this->botrate->addItem("115200");
    this->botrate->addItem("9600");
    this->botrate->addItem("19200");
    this->botrate->addItem("38400");
    this->botrate->addItem("57600");//参见QT助手和UB482用户手册可用波特率

    this->datasize->addItem("8");//同上，可用数据位

    this->stopsize->addItem("1");//同上，可用停止位

    this->checksize->addItem("无校验");
    this->checksize->addItem("奇校验");
    this->checksize->addItem("偶校验");

    this->resMode->addItem("ASCII");//文本指令接收
    this->resMode->addItem("HEX");//二进制指令接收

    this->sendMode->addItem("ASCII");
    this->sendMode->addItem("HEX");


    //为每个combox设置标签
    QLabel* portnumlabel = new QLabel("串口编号", this);
    QLabel* botratelabel = new QLabel("波特率", this);
    QLabel* datasizelabel = new QLabel("数据位", this);
    QLabel* stopsizelabel = new QLabel("停止位", this);
    QLabel* checksizelabel = new QLabel("校验位", this);
    QLabel* resmodelabel = new QLabel("接收模式", this);
    QLabel* sendmodelabel = new QLabel("发送模式", this);

    //构建combox和lable容器列表
    QVector<QComboBox*>setups;
    setups.push_back(portnum);
    setups.push_back(botrate);
    setups.push_back(datasize);
    setups.push_back(stopsize);
    setups.push_back(checksize);
    setups.push_back(sendMode);
    setups.push_back(resMode);

    QVector<QLabel*>labels;
    labels.push_back(portnumlabel);
    labels.push_back(botratelabel);
    labels.push_back(datasizelabel);
    labels.push_back(stopsizelabel);
    labels.push_back(checksizelabel);
    labels.push_back(sendmodelabel);
    labels.push_back(resmodelabel);

    //循环创建布局
    for (int i = 0; i < setups.size(); i++) {
        setups[i]->setFixedSize(150 * wb, 50 * hb);
        setups[i]->move(150*wb, (20 + i * 80)*hb);
        labels[i]->setFixedSize(100 * wb, 50 * hb);
        labels[i]->move(30 * wb, (25 + i * 80) * hb);
    }
}

#pragma 配置串口命令
void UBSerials::Serial_CMD_System(void) {
    serialPort->write("unlogall\r\n");
    serialPort->write("mask gps\r\n");
    serialPort->write("mask bds\r\n");
    serialPort->write("mask glo\r\n");
    serialPort->write("mask gal\r\n");
    serialPort->write("mask qzss\r\n");
    if (Systemselect->currentText() == "BDS") {
        serialPort->write("unmask B1\r\n");
        serialPort->write("log bd2ephemb onchanged\r\n");
        serialPort->write("log bd2ionutcb once\r\n");
    }
    if (Systemselect->currentText() == "GPS") {
        serialPort->write("unmask L1\r\n");
        serialPort->write("log gpsephemb onchanged\r\n");
        serialPort->write("gpsionb onchanged\r\n");
    }
    serialPort->write("log rangeb ontime 1\r\n");
    serialPort->write("log psrposb ontime 1\r\n");
    serialPort->write("BDSUTCB\r\n");
}

#pragma 可用串口列表/定权模型更新（更新率1HZ）
void UBSerials::Refreshports(void) {
    QVector<QString>temp;
    for (const QSerialPortInfo& info : QSerialPortInfo::availablePorts()) {
        temp.push_back(info.portName());
    }
    //std::sort(temp.begin(), temp.end());
    if (temp != this->ports) {
        this->portnum->clear();
        this->ports = temp;
        for (auto& aport : ports) this->portnum->addItem(aport);
    }
}
void UBSerials::Refreshfitmodel(void) {
    if (Fitmodel->currentText() == "单位阵")
        Fitmodelnum = 0;
    else if (Fitmodel->currentText() == "高度角1")
        Fitmodelnum = 1;
    else if (Fitmodel->currentText() == "高度角2")
        Fitmodelnum = 2;
    else if (Fitmodel->currentText() == "伪距标准差")
        Fitmodelnum = 3;
    else if (Fitmodel->currentText() == "L1信噪比")
        Fitmodelnum = 4;
    else if (Fitmodel->currentText() == "信噪比")
        Fitmodelnum = 5;
}
void UBSerials::timerEvent(QTimerEvent* e) {
    if(e->timerId()==portnum_TimeID)
    {
        Refreshports();//更新串口列表
        Refreshfitmodel();//更新定权模型
    }
    if (e->timerId() == painter_TimeID) {
        repaint();
    }
}

#pragma 串口连接与数据获取
void UBSerials::StartUSART(void) {

    //初始化“打开串口”和“关闭串口”按钮对象
    startUSART = new QPushButton("打开串口", this);
    endUSART = new QPushButton("关闭串口", this);
    startUSART->setFixedSize(270 * wb, 50 * hb);
    endUSART->setFixedSize(270 * wb, 50 * hb);
    startUSART->move(30*wb, 570*hb);
    endUSART->move(30*wb, 630*hb);

    //断开连接槽函数
    endUSART->setDisabled(true);

    connect(endUSART, &QPushButton::clicked, [&]() {
        //发送、断连按钮失效，连接按钮复位
        sendButton->setDisabled(true);
        startUSART->setDisabled(false);
        endUSART->setDisabled(true);
        //断连操作
        serialPort->close();
        });

    //打开连接槽函数
    connect(startUSART, &QPushButton::clicked, [&]() {
        //连接按钮失效，断连、发送按钮复位
        startUSART->setDisabled(true);
        endUSART->setDisabled(false);
        sendButton->setDisabled(false);
        //连接操作
        if (portnum->currentText() != "")
            USART();//连接串口函数
        else
            QMessageBox::critical(this, "串口打开失败", "请确认串口是否正确连接");
        });

}
void UBSerials::USART(void) {
    QSerialPort::BaudRate Baud;//波特率
    QSerialPort::DataBits Data;//数据位
    QSerialPort::StopBits Stop;//停止位
    QSerialPort::Parity check;//校验

    //设置波特率
    if (this->botrate->currentText() == "115200")
        Baud = QSerialPort::Baud115200;
    else if (this->botrate->currentText() == "9600")
        Baud = QSerialPort::Baud9600;
    else if (this->botrate->currentText() == "19200")
        Baud = QSerialPort::Baud19200;
    else if (this->botrate->currentText() == "38400")
        Baud = QSerialPort::Baud38400;
    else if (this->botrate->currentText() == "57600")
        Baud = QSerialPort::Baud57600;

    //数据位设置
    if (this->datasize->currentText() == "8") Data = QSerialPort::Data8;

    //停止位设置
    if (this->stopsize->currentText() == "1") Stop = QSerialPort::OneStop;
    
    //校验位设置
    if (this->checksize->currentText() == "无校验") check = QSerialPort::NoParity;
    else if (this->checksize->currentText() == "奇校验") check = QSerialPort::OddParity;
    else if (this->checksize->currentText() == "偶校验") check = QSerialPort::EvenParity;

    //串口对象初始化与连接
    serialPort = new QSerialPort(this);
    serialPort->setBaudRate(Baud);
    serialPort->setDataBits(Data);
    serialPort->setStopBits(Stop);
    serialPort->setParity(check);
    serialPort->setPortName(this->portnum->currentText());//串口号
    serialPort->setReadBufferSize(BUFFSIZE);
    
    //设置串口对象为可读可写
    if (serialPort->open(QSerialPort::ReadWrite)) {
        connect(serialPort, &QSerialPort::readyRead, [&]() {
            slen = serialPort->bytesAvailable();//可用数据长度
            
            //原始数据获取
            auto data = serialPort->readAll();
            
            //串口接收区的两种模式显示
            if (resMode->currentText() == "HEX") {      //字节模式
                QString hex = data.toHex(' ');
                receiveArea->appendPlainText(hex);
            }
            else {                                          //文本模式
                QString str = QString(data);
                receiveArea->appendPlainText(str);
            }
            
            //自解算
            memset(buffer, 0, 8192);//缓冲区清空
            for (int i = 0; i < slen; i++)
            {
                buffer[i] = unsigned char(data[i]);
                buff.buff[i] = buffer[i];
            }
            
            if (realtimesolveflag == 1 && Saveselect->currentText() == "保存")
                fwrite(buffer, 1, slen, fp);
            
            if (realtimesolveflag == 1) {
                int messagex=RefreshUsedmessage(message);
                computing(message, messagex);//自解算线程
            }
           
            
            });
    }else {
        QMessageBox::critical(this, "串口打开失败", "请确认串口是否正确连接");
        sendButton->setDisabled(true);
        startUSART->setDisabled(false);
        endUSART->setDisabled(true);
    };
}

#pragma 自解算线程函数
int UBSerials::RefreshUsedmessage(unsigned char* message) {
    
    buff.bufflen = slen;
    //构建有效数据区
    int messagex = 0;
    
    memset(message, 0x00, sizeof(message));//将上一次使用的有效数据区清空
    memcpy(message, Left.buff, Left.bufflen);//复制上一次剩余的数据
    memcpy(message + Left.bufflen, buff.buff, buff.bufflen);//复制本次新增的数据
    messagex = buff.bufflen + Left.bufflen;//本次总数据长度
    
    
    //从有效数据区中获取消息报告
    breport epoch[400] = {};
    int epochnum = 0;
    epochnum = getbinaryreport(epoch, message, messagex);
    memset(Left.buff, 0x00, sizeof(Left.buff)); Left.bufflen = 0;//上一次剩余数据区清零
    Left.bufflen = messagex - epoch[epochnum].start;//重新构建剩余数据区结构(定长)
    memcpy(Left.buff, message + epoch[epochnum].start, Left.bufflen);//重新构建剩余数据区

    return messagex;//将有效数据区长度作为返回值返回
}
void UBSerials::computing(unsigned char* message, int messagex) {
    
    //获取二进制消息报告
    breport epoch[400];
    int epochnum = 0;
    epochnum = getbinaryreport(epoch, message, messagex);

    //定位主循环
    for (int j = 0; j < epochnum; j++) {

        //GPS星历
        if (epoch[j].ID == 7) {
            int prn;
            prn = getsat(eph, message, epoch[j]);
            eph[prn].statu = EPHYES;
        }
        //北斗B1信号星历
        if (epoch[j].ID == 1047) {
            int prn;
            prn = getbds2eph(bds, message, epoch[j]);
            bds[prn].statu = EPHYES;
        }
        //GPS电离层，因为老师提供的文件有两种板卡协议，0927数据一定要加上"OEM7"参数调用重载，Com_16不需要
        if (epoch[j].ID == 8) {
            getion(message, ion, epoch[j]);
        }
        //以下为BDS电离层参数，需调用重载
        if (epoch[j].ID == 2010) {
            getion(message, ion_bds, epoch[j], "BDS");
        }
        //以下为BDS与UTC之间的整秒差，用于授时改正
        if (epoch[j].ID == 2012) {
            bdt_utc = bit2long(message + epoch[j].start + 24 + 32);
        }
        //如果读到观测值，则进行定位尝试，ID=631为OEM7协议，ID=43为UB482协议
        if (epoch[j].ID == 631 || epoch[j].ID == 43) {
            
           
            getobs(R, message, epoch[j]);//获取观测值
            
            //GPS单系统单点定位
            if (tobsnow.time != R.rt.time && Systemselect->currentText()=="GPS") {
                int flag = SPPpos(R, eph, ion, Result, Fitmodelnum);
                tobsnow.time = R.rt.time; tobsnow.second = R.rt.second;//更新系统时间
                sprintf(Result.sysytem, "GPS");
                if (flag >= 4) {
                    char str[200] = {}; double blh[3] = {};
                    xyztoblh(Result.xyzt[0], Result.xyzt[1], Result.xyzt[2], blh);
                    double Vspeed = (Result.xyztspeed[0] * Result.xyztspeed[0])
                        + (Result.xyztspeed[0] * Result.xyztspeed[0])
                        + (Result.xyztspeed[0] * Result.xyztspeed[0]);
                    Vspeed = sqrt(Vspeed);
                    sprintf(str, "From GPS:\n Lat:%lf\n Lon:%lf\n Hight:%lf\n Speed:%lf\n GPST:\%lld", blh[0], blh[1], blh[2], Vspeed, Result.tobs.time);
                    if (Saveselect->currentText() == "保存")
                        fprintResult(fs, Result);
                    this->computingshowArea->appendPlainText(QString(str));
                    mapmarker(blh[1], blh[0]);
                }
                else
                {
                    char str[200] = {}; sprintf(str, "GPST:%lld\n Less than 4 sats could be seen\n", R.rt.time);
                    this->computingshowArea->appendPlainText(QString(str));
                }
            }
            //GPS单系统单点定位结束
            
            //BDS单系统单点定位
            if (tobsnow.time != R.rt.time && Systemselect->currentText()=="BDS") {
                int flag_bds = SPPpos(R, bds, ion_bds, Result, Fitmodelnum);
                tobsnow.time = R.rt.time; tobsnow.second = R.rt.second;//更新系统时间
                sprintf(Result.sysytem, "BDS");
                if (flag_bds >= 4) {
                    char str[200] = {}; double blh[3] = {};
                    xyztoblh(Result.xyzt[0], Result.xyzt[1], Result.xyzt[2], blh);
                    double Vspeed = (Result.xyztspeed[0] * Result.xyztspeed[0])
                        + (Result.xyztspeed[0] * Result.xyztspeed[0])
                        + (Result.xyztspeed[0] * Result.xyztspeed[0]);
                    Vspeed = sqrt(Vspeed);
                    sprintf(str, "From BDS:\n Lat:%lf\n Lon:%lf\n Hight:%lf\n Speed:%lf\n GPST:\%lld", blh[0], blh[1], blh[2], Vspeed, Result.tobs.time);
                    if (Saveselect->currentText() == "保存")
                        fprintResult(fs, Result);
                    this->computingshowArea->appendPlainText(QString(str));
                    mapmarker(blh[1], blh[0]);
                }
                else
                {
                    char str[200] = {}; sprintf(str, "GPST:%lld\n Less than 4 sats could be seen\n", R.rt.time);
                    this->computingshowArea->appendPlainText(QString(str));
                }
            }
            //BDS单系统定位结束
        }
        
        //输出接收机原始结果
        if (epoch[j].ID == 47) {
            int start = epoch[j].start;
            double lat = bit2double(message + start + 28 + 8);
            double lon = bit2double(message + start + 28 + 16);
            double he = bit2double(message + start + 28 + 24) + bit2float(message + start + 28 + 32);
            //注意接收机直接输出的是海拔高，需要与其内置的水准面模型相加才能得到直接解算的椭球高he
            gtime_t ubobstime = {};
            ubobstime = gpst2time(int(breport2GPStime(epoch[j], message).Week), breport2GPStime(epoch[j], message).Second);
            //自解算的历元时间
            char str[200] = {}; sprintf(str, "From UB482 %lf %lf %lf\n", lat, lon, he);
            if (ubobstime.time == tobsnow.time)
                this->computingshowArea->appendPlainText(QString(str));
        }
    }
}

#pragma 高德地图显示
void UBSerials::mapshowInit(void) {
    Gaodemapshow = new QWebEngineView(this);
    Gaodemapshow->setFixedSize(800 * wb, 800 * hb);
    Gaodemapshow->move(1150*wb, 20*hb);
    Gaodemapshow->setZoomFactor(1.5 * wb);

    QNetworkProxyFactory::setUseSystemConfiguration(false);
    Gaodemapshow->page()->load(QUrl(htmlFilePath));
    Gaodemapshow->show();

    QPushButton* mapchange = new QPushButton("地图切换", this);
    mapchange->setFixedSize(100 * wb, 40 * hb);
    mapchange->move(1850 * wb, 790 * hb);
    connect(mapchange, &QPushButton::clicked, [&]() {
        if (htmlFilePath.contains("Gaode", Qt::CaseInsensitive)) {
            htmlFilePath = QApplication::applicationDirPath() + "/mapsource/Baidumap.html";
            Gaodemapshow->page()->load(QUrl(htmlFilePath));
        }
        else {
            htmlFilePath = QApplication::applicationDirPath() + "/mapsource/Gaodemap.html";
            Gaodemapshow->page()->load(QUrl(htmlFilePath));
        }

            
        });
    
    QPushButton* Refreshmap = new QPushButton("地图重置", this);
    Refreshmap->setFixedSize(150 * wb, 50 * hb);
    Refreshmap->move(1150*wb, 830*hb);

    connect(Refreshmap, &QPushButton::clicked, [&]() {
        if(htmlFilePath.contains("Gaode",Qt::CaseInsensitive))
            Gaodemapshow->page()->runJavaScript("repoint([116.397428, 39.90923]);");//高德地图
        else
            Gaodemapshow->page()->runJavaScript("repoint([116.404, 39.915]);");//百度地图
        });
    
    Realtimemap = new QPushButton("实时展示",this);
    Realtimemap->setFixedSize(150 * wb, 50 * hb);
    Realtimemap->move(1320*wb, 830*hb);
    Realtimemap->setDisabled(false);
    
    shutRealtimemap = new QPushButton("关闭实时展示", this);
    shutRealtimemap->setFixedSize(150 * wb, 50 * hb);
    shutRealtimemap->move(1490*wb, 830*hb);
    shutRealtimemap->setDisabled(true);
    
    connect(Realtimemap, &QPushButton::clicked, [&]() {
        if (serialPort != NULL&&serialPort->isOpen()) {
            map_TimeID = this->startTimer(1000);
            Realtimemap->setDisabled(true);
            shutRealtimemap->setDisabled(false);
        }
        else {
            QMessageBox::critical(this, "串口打开失败", "请确认串口是否正确连接");
        }
        
        });

    connect(shutRealtimemap, &QPushButton::clicked, [&]() {
        map_TimeID = 0;//重置进程计时器编号
        killTimer(map_TimeID);
        shutRealtimemap->setDisabled(true);
        Realtimemap->setDisabled(false);
        });

    QPushButton* clearAllmakers = new QPushButton("清除所有标记点", this);
    clearAllmakers->setFixedSize(250 * wb, 50 * hb);
    clearAllmakers->move(1700*wb, 830*hb);

    connect(clearAllmakers, &QPushButton::clicked, [&]() {
        Gaodemapshow->page()->runJavaScript("clearmap();");//清除所有标记
        });

    QPushButton* Singlepoint = new QPushButton("单点标记", this);
    Singlepoint->setFixedSize(200 * wb, 40 * hb);
    Singlepoint->move(1750*wb, 890*hb);

    Latedit = new QPlainTextEdit(this);
    Latedit->setFixedSize(200 * wb, 40 * hb);
    Latedit->move(1500*wb, 890*hb);
    QLabel* Latlabel = new QLabel("Lat:", this);
    Latlabel->setFixedSize(50 * wb, 40 * hb);
    Latlabel->move(1450*wb, 890*hb);

    Lngedit = new QPlainTextEdit(this);
    Lngedit->setFixedSize(200 * wb, 40 * hb);
    Lngedit->move(1200*wb, 890*hb);
    QLabel* Lnglabel = new QLabel("Lng:", this);
    Lnglabel->setFixedSize(50 * wb, 40 * hb);
    Lnglabel->move(1150*wb, 890*hb);

    connect(Singlepoint, &QPushButton::clicked, [&]() {
        if (Latedit->toPlainText().isEmpty() || Lngedit->toPlainText().isEmpty())
        {
            QMessageBox::critical(this, "经纬度格式错误", "请输入正确的经度(Lng)和纬度(Lat)"); 
            return; 
        }
        double lat = 0, lng = 0;
        char str[100] = {};
        lat = Latedit->toPlainText().toDouble(), lng = Lngedit->toPlainText().toDouble();
        double gcj_02_pos[2] = {};
        wgs84_to_gcj02(lng, lat, gcj_02_pos);
        sprintf(str, "makerpoint([%lf,%lf])", gcj_02_pos[0], gcj_02_pos[1]);
        Gaodemapshow->page()->runJavaScript(str);
        
        });
}
void UBSerials::mapmarker(double lng, double lat) {
    double gcj02pos[2] = {};
    if (map_TimeID == 0)
        return;
    wgs84_to_gcj02(lng, lat, gcj02pos);
    char str[100] = {};
    sprintf(str, "point([%lf,%lf]);", gcj02pos[0], gcj02pos[1]);
    Gaodemapshow->page()->runJavaScript(str);
}

#pragma 绘图区初始化与显示
void UBSerials::paintEvent(QPaintEvent* e) {
    azelShow = new QPainter(this);
  
    QPen pen;
    pen.setWidth(3);
    pen.setColor(Qt::black);
    pen.setStyle(Qt::SolidLine);
    pen.setCapStyle(Qt::FlatCap);
    pen.setJoinStyle(Qt::BevelJoin);
    azelShow->setPen(pen);
    /*
    azelShow->drawText(QPoint(865*wb, 890*hb), "75");
    azelShow->drawText(QPoint(865*wb, 850*hb), "60");
    azelShow->drawText(QPoint(865*wb, 810*hb), "45");
    azelShow->drawText(QPoint(865*wb, 770*hb), "30");
    azelShow->drawText(QPoint(865*wb, 730*hb), "15");
    *///显示效果问题，暂时关闭高度角图例
    for(int i=0;i<5;i++)
        azelShow->drawEllipse(QPoint(875*wb, 930*hb),int(40*(i+1)*wb), int(40*(i+1)*wb));
    qDebug() << 40 * (1) * wb << int(40 * (1) * wb);
    //绘制可视可用卫星方位角高度角图
    QFont font;
    font.setPixelSize(16*wb);
    font.setBold(true);
    azelShow->setFont(font);
    azelShow->translate(875 * wb, 930 * hb);
    for (int i = 0; i < R.num; i++) {
        int prnname = R.name[i];
        double taz = 0.0, tel = 0.0;
        if (Systemselect->currentText() == QString("BDS"))
            taz = azel_BDS[prnname][0], tel = azel_BDS[prnname][1];
        else if(Systemselect->currentText()==QString("GPS"))
            taz = azel_GPS[prnname][0], tel = azel_GPS[prnname][1];
  
        double r = hb * 240 * (1 - 2 * tel / pi);
        azelShow->drawEllipse(QPoint(r * sin(taz), -r * cos(taz)), int(12 * hb), int(12 * hb));
        QRect trect(r * sin(taz) - 12*hb, -r * cos(taz) - 12*hb, 24*hb, 24*hb);
        azelShow->drawText(trect, Qt::AlignCenter, QString("%1").arg(prnname));
    }
    azelShow->resetTransform();
}

#pragma LCD时间显示
void UBSerials::dateLCDInit(void){
    dateshow = new QLCDNumber(this);
    dateshow->setDigitCount(19);
    dateshow->setFixedSize(470 * wb, 50 * hb);
    dateshow->move(650*wb, 630*hb);
    dateshow->display("1970-01-01 00:00:00");

    QTimer* lcdtimer = new QTimer(this);
    connect(lcdtimer, &QTimer::timeout, [&]() {
        if (!realtimesolveButton->isEnabled()) {
            char showtime[30] = {};
            COMMONTIME t = {};
            gtime_t utc = R.rt;
            utc.time -= 14;//换算为北斗时
            utc.time = utc.time - bdt_utc;
            t = time2epoch(utc);
            sprintf(showtime, "%4d-%02d-%02d %02d:%02d:%02.0lf", t.Year, t.Month, t.Day, t.Hour, t.Minute, t.Second);
            dateshow->display(showtime);
        }
        });
    lcdtimer->start(50);
}
void UBSerials::RefreshLCDtime(void) {
    if (realtimesolveButton->isEnabled()) {
        char showtime[30] = {};
        COMMONTIME t = {};
        t = time2epoch(R.rt);
        sprintf(showtime, "%4d-%02d-%02d %02d:%02d:%02.0lf", t.Year, t.Month, t.Day, t.Hour, t.Minute, t.Second);
        dateshow->display(showtime);
    }
}

#pragma 文件解算区
void UBSerials::File_solveInit(void) {
    OBSfilechoice = new QPlainTextEdit(this);//观测值文件选择
    SATfilechoice = new QPlainTextEdit(this);//星历文件选择
    QLabel* OBSClabel = new QLabel("观测值文件", this);
    QLabel* SATClabel = new QLabel("星历文件", this);
    QPushButton* obsselectbt = new QPushButton("...", this);
    QPushButton* satselectbt = new QPushButton("...", this);
    
    Fileselect = new QComboBox(this);
    QLabel* FSelectlabel = new QLabel("文件格式", this);

    File_system_select = new QComboBox(this);
    QLabel* F_sys_Selectlabel = new QLabel("解算模式", this);
    File_system_select->addItem("卫星位置");
    File_system_select->addItem("测站位置");
    
    Fileselect->addItem("RINEX");
    Fileselect->addItem("二进制");

    OBSfilechoice->setFixedSize(600 * wb, 40 * hb);
    OBSfilechoice->move(1150 * wb, 970 * hb);
    OBSClabel->setFixedSize(200 * wb, 40 * hb);
    OBSClabel->move(1800 * wb, 970 * hb);
    obsselectbt->setFixedSize(40 * wb, 40 * hb);
    obsselectbt->move(1750 * wb, 970 * hb);

    SATfilechoice->setFixedSize(600*wb, 40*hb);
    SATfilechoice->move(1150 * wb, 1020 * hb);
    SATClabel->setFixedSize(200 * wb, 40 * hb);
    SATClabel->move(1800 * wb, 1020 * hb);
    satselectbt->setFixedSize(40 * wb, 40 * hb);
    satselectbt->move(1750 * wb, 1020 * hb);

    Fileselect->setFixedSize(100 * wb, 50 * hb);
    Fileselect->move(1150 * wb, 1080 * hb);
    FSelectlabel->setFixedSize(200 * wb, 50 * hb);
    FSelectlabel->move(1250 * wb, 1080 * hb);
    File_system_select->setFixedSize(130 * wb, 50 * hb);
    File_system_select->move(1350 * wb, 1080 * hb);
    F_sys_Selectlabel->setFixedSize(200 * wb, 50 * hb);
    F_sys_Selectlabel->move(1480 * wb, 1080 * hb);

    connect(obsselectbt, &QPushButton::clicked, [&]() {
        QString currentpath = QDir::currentPath() + "/Example/";//当前路径
        QString dlgtitle = "选择观测文件";//文件对话框名称
        QString filter = "所有文件(*.*) ;; 文本文件(*.txt)";//文件显示配置
        QString fileName = QFileDialog::getOpenFileName(this, dlgtitle, currentpath, filter);
        if (!fileName.isEmpty())
            OBSfilechoice->setPlainText(fileName);
        });
    
    connect(satselectbt, &QPushButton::clicked, [&]() {
        QString currentpath = QDir::currentPath() + "/Example/";//当前路径
        QString dlgtitle = "选择星历文件";//文件对话框名称
        QString filter = "所有文件(*.*) ;; 文本文件(*.txt)";//文件显示配置
        QString fileName = QFileDialog::getOpenFileName(this, dlgtitle, currentpath, filter);
        if (!fileName.isEmpty())
            SATfilechoice->setPlainText(fileName);
        });

    QPushButton* Filesolvebt = new QPushButton("文件解算", this);
    Filesolvebt->setFixedSize(150*wb, 50*hb);
    Filesolvebt->move(1580*wb, 1080*hb);
    connect(Filesolvebt, &QPushButton::clicked, this ,&UBSerials::FilesolvepageInit);

    QPushButton* Stagesolvebt = new QPushButton("中间值解算", this);
    Stagesolvebt->setFixedSize(200*wb, 50*hb);
    Stagesolvebt->move(1750*wb, 1080*hb);
    connect(Stagesolvebt, &QPushButton::clicked, [&]() {
        children.show();
        });
}
void UBSerials::FilesolvepageInit(void) {
    /*基础设置*/
    Filesolvepage = new QWidget();
    Filesolvepage->setWindowTitle("文件解算");
    Filesolvepage->setFixedSize(1600*wb,800*hb);
    solveresultshow = new QPlainTextEdit(Filesolvepage);
    solveresultshow->setReadOnly(true);
    solveresultshow->setFixedSize(1500*wb, 700*hb);
    solveresultshow->move(50*wb, 80*hb);
    
    /*解算函数*/
    QLabel* showlabel_1 = new QLabel("解算中", Filesolvepage);
    showlabel_1->setFixedSize(1500*wb, 40*hb); showlabel_1->setAlignment(Qt::AlignCenter);
    QFont font; font.setPixelSize(40*wb); showlabel_1->setFont(font);
    showlabel_1->move(50*wb, 30*hb);
    
    replaybt = new QPushButton("轨迹重现", Filesolvepage);
    replaybt->setFixedSize(200 * wb, 50 * hb);
    replaybt->move(1350 * wb, 30 * hb);
    font.setPixelSize(24 * wb);
    replaybt->setFont(font);
    solveresultshow->setFont(font);
    Filesolvepage->show();
    
    int flag = File_SAT_POS();

    /*解算结果显示*/
    if (flag == 1)
    {
        showlabel_1->setText("RINEX文件解算GPS卫星位置结果");
    }
    else if (flag == 2)
    {
        showlabel_1->setText("RINEX文件解算BDS卫星位置结果");
    }
    else if (flag == 5) {
        showlabel_1->setText("RINEX文件GPS测站单点定位测速结果");
    }
    else if (flag == 6) {
        showlabel_1->setText("RINEX文件BDS测站单点定位测速结果");
    }
    else if (flag == 3) {
        showlabel_1->setText("二进制文件解算GPS卫星位置结果");
    }
    else if (flag == 4) {
        showlabel_1->setText("二进制文件解算BDS卫星位置结果");
    }
    else if (flag == 7) {
        showlabel_1->setText("二进制文件GPS单系统单点定位测速结果");
    }
    else if (flag == 8) {
        showlabel_1->setText("二进制文件BDS单系统单点定位测速结果");
    }
    else if (flag == 0) {
        QMessageBox::critical(this, "解算失败", "请检查输入文件和解算选项后重试");
        Filesolvepage->close();
        return;
    }
    /*轨迹绘制槽函数*/
    if (flag == 1 || flag == 2 || flag == 3 || flag == 4) {
        delete replaybt;
    }

    connect(replaybt, &QPushButton::clicked, [&]() {
        QString result = solveresultshow->toPlainText();//读取解算结果
        QStringList lines = result.split(QRegExp("[\r\n]"), Qt::SkipEmptyParts);
        for (int i = 1; i < lines.size(); i++) {
            char* str = lines[i].toLatin1().data();
            if (strstr(str, "GPST"))
                continue;
            double t = 0.0, lon = 0.0, lat = 0.0; int num = 0;
            sscanf(str, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %d", &t, &t, &t, &t, &t, &t, &t, &t, &lat, &lon, &t, &num);
            
            if (num) {
                double gcj_02_pos[2] = {};
                wgs84_to_gcj02(lon, lat, gcj_02_pos);
                sprintf(str, "makerpoint([%lf,%lf])", gcj_02_pos[0], gcj_02_pos[1]);
                Gaodemapshow->page()->runJavaScript(str);
            }
        }
        });
    
 }


int UBSerials::File_SAT_POS(void) {
    //首先检查文件路径是否正确
    FILE* satf;
    if (!(satf = fopen(OBSfilechoice->toPlainText().toLocal8Bit().data(), "r"))) {
        QMessageBox::critical(this, "观测值文件打开失败", "请检查观测值文件路径是否正确");
        return 0;
    }
    fclose(satf);
    if (!(satf = fopen(SATfilechoice->toPlainText().toLocal8Bit().data(), "r"))) {
        QMessageBox::critical(this, "星历文件打开失败", "请检查星历文件路径是否正确");
        return 0;
    }
    fclose(satf); 
    
    if (File_system_select->currentText() == "卫星位置") {

        if (Fileselect->currentText() == "RINEX")
        {
            if (Systemselect->currentText() == "GPS")
            {
                solveresultshow->appendPlainText("PRN X Y Z X_speed Y_speed Z_speed Dt Dt_speed");
                RINEX_GPS_sat(SATfilechoice->toPlainText().toLocal8Bit().data(), OBSfilechoice->toPlainText().toLocal8Bit().data());
                return 1;//RINEX_GPS卫星位置
            }
            else if (Systemselect->currentText() == "BDS")
            {
                solveresultshow->appendPlainText("PRN X Y Z X_speed Y_speed Z_speed Dt Dt_speed");
                RINEX_BDS_sat(SATfilechoice->toPlainText().toLocal8Bit().data(), OBSfilechoice->toPlainText().toLocal8Bit().data());
                return 2;//RINEX_北斗卫星位置
            }
            
        }
        if (Fileselect->currentText() == "二进制") {
            if (SATfilechoice->toPlainText() != OBSfilechoice->toPlainText())
                {
                    QMessageBox::critical(this, "二进制文件不匹配", "请在观测值文件和星历文件选择框中输入相同的文件路径");
                    return 0;
                }
            
            if (Systemselect->currentText() == "GPS") {    
                solveresultshow->appendPlainText("PRN X Y Z X_speed Y_speed Z_speed Dt Dt_speed");
                BINEARY_sat(SATfilechoice->toPlainText().toLocal8Bit().data(), 1);
                return 3;//GPS二进制卫星位置
            }
            else if (Systemselect->currentText() == "BDS") {
                solveresultshow->appendPlainText("PRN X Y Z X_speed Y_speed Z_speed Dt Dt_speed");
                BINEARY_sat(SATfilechoice->toPlainText().toLocal8Bit().data(), 3);
                return 4;//BDS二进制卫星位置
            }
        };
    }
    else {
        if (Fileselect->currentText() == "RINEX")
        {
            if (Systemselect->currentText() == "GPS") {
                solveresultshow->appendPlainText("X Y Z  Clock_Dt X_speed Y_speed Z_speed Clock_Dt_speed B L H satsum");
                RINEX_GPS_sat(SATfilechoice->toPlainText().toLocal8Bit().data(), OBSfilechoice->toPlainText().toLocal8Bit().data(), 2);
                return 5;//RINEX_GPS单点定位与测速
            }
            else if (Systemselect->currentText() == "BDS") {
                solveresultshow->appendPlainText("X Y Z  Clock_Dt X_speed Y_speed Z_speed Clock_Dt_speed B L H satsum");
                RINEX_BDS_sat(SATfilechoice->toPlainText().toLocal8Bit().data(), OBSfilechoice->toPlainText().toLocal8Bit().data(), 2);
                return 6;//RINEX北斗单点定位与测速
            }
        }
        if (Fileselect->currentText() == "二进制") {
            if (SATfilechoice->toPlainText() != OBSfilechoice->toPlainText())
            {
                QMessageBox::critical(this, "二进制文件不匹配", "请在观测值文件和星历文件选择框中输入相同的文件路径");
                return 0;
            }

            if (Systemselect->currentText() == "GPS") {
                solveresultshow->appendPlainText("X Y Z  Clock_Dt X_speed Y_speed Z_speed Clock_Dt_speed B L H satsum");
                BINEARY_sat(SATfilechoice->toPlainText().toLocal8Bit().data(), 2);
                return 7;//GPS二进制测站单点定位测速
            }
            else if (Systemselect->currentText() == "BDS") {
                solveresultshow->appendPlainText("X Y Z  Clock_Dt X_speed Y_speed Z_speed Clock_Dt_speed B L H satsum");
                BINEARY_sat(SATfilechoice->toPlainText().toLocal8Bit().data(), 4);
                return 8;//BDS二进制单点定位测速
            }
        }
    }

}

/*以下为从RINEX文件中直接解出卫星位置的函数*/

void UBSerials::RINEX_GPS_sat(const char* SAT_filename, const char* OBS_filename, int mode) {
    FILE* fp;
    fp = fopen(SAT_filename, "r");
    char str[100] = {}; int count = 0; int name[36] = {}, namecount = 0;
    while (!feof(fp))
    {
        fgets(str, 100, fp);
        if (strstr(str, "END OF HEADER"))
            break;
    }
    while (!feof(fp))
    {
        fgets(str, 100, fp); count++;
        if (count % 8 == 1)
            sscanf(str, "%d", &name[namecount++]);
    }

    fclose(fp);//星历文件的卫星prn号读取完毕，下面开始获取星历

    eph_t GPS[36] = {};
    double data[32] = {}; int snum[9] = {};
    for (int i = 0; i < namecount; i++)
    {
        readmessagefile(name[i], SAT_filename, data, snum);
        GPS[name[i]] = getsate(data);
        GPS[name[i]].statu = EPHYES;
    }//星历构建完成，下面构建电离层参数
    double ion[8] = {};
    fp = fopen(SAT_filename, "r");
    while (!feof(fp)) {
        fgets(str, 100, fp);
        if (strstr(str, "ION ALPHA")) {
            charreplace(str, 'D', 'E');
            sscanf(str, "%lf %lf %lf %lf", &ion[0], &ion[1], &ion[2], &ion[3]);
        }
        if (strstr(str, "ION BETA")) {
            charreplace(str, 'D', 'E');
            sscanf(str, "%lf %lf %lf %lf", &ion[4], &ion[5], &ion[6], &ion[7]);
        }
    }
    fclose(fp);

    //电离层参数构建完成，下面构建观测值
    fp = fopen(OBS_filename, "r");
    while (!feof(fp)) {
        fgets(str, 100, fp);
        if (strstr(str, "END OF HEADER"))
            break;
    }
    while (!feof(fp)) {
        fgets(str, 100, fp);
        if (strstr(str, ">"))
        {
            GPSOBS R = {};
            COMMONTIME ct = {}; int epoch_sat_num = 0, no = 0;
            sscanf(str, ">%d %d %d %d %d %lf %d %d", &ct.Year, &ct.Month, &ct.Day, &ct.Hour, &ct.Minute, &ct.Second, &no, &epoch_sat_num);
            R.rt = com2unixtime(ct);//获取观测历元时间
            QString toplain_ct;
            toplain_ct.sprintf("GPSTime:%4d.%02d.%02d %02d:%02d:%lf", ct.Year, ct.Month, ct.Day, ct.Hour, ct.Minute, ct.Second);
            solveresultshow->appendPlainText(toplain_ct);
            if (no != 0)
                continue;//观测值无效
            char rrstr[400] = {}; int k = 0;
            //在一个历元的所有观测值中获取GPS系统L1信号观测值
            for (int j = 0; j < epoch_sat_num; j++) {
                fgets(rrstr, 400, fp);
                if (strstr(rrstr, "G"))//读到GPS观测值
                {
                    char substr[100] = {};
                    memcpy(substr, rrstr, 3);
                    sscanf(substr, "G%d", &R.name[k]);
                    memset(substr, '\0', sizeof(substr));
                    memcpy(substr, rrstr + 3, 14);
                    sscanf(substr, "%lf", &R.R0[k]);
                    memset(substr, '\0', sizeof(substr));
                    memcpy(substr, rrstr + 17, 16);
                    sscanf(substr, "%lf", &R.adr[k]);
                    memset(substr, '\0', sizeof(substr));
                    memcpy(substr, rrstr + 35, 14);
                    sscanf(substr, "%f", &R.dopp[k]);
                    memset(substr, '\0', sizeof(substr));
                    memcpy(substr, rrstr + 49, 16);
                    sscanf(substr, "%f", &R.Cno[k]);
                    if (R.R0[k] || R.adr[k] || R.dopp[k])//有观测值则取值
                        k++;
                }

            }
            //观测值读取完毕，以下开始计算卫星位置
            R.num = k;//该历元GPS单系统观测值数量

            if (mode == 1) {
                double xyz[3] = {}, dotxyz[3] = {}, tdts = 0.0, tdtss = 0.0;
                for (int i = 0; i < R.num; i++) {
                    getsatelliteposition(GPS[R.name[i]], 'G', R.name[i], R.R0[i], xyz, dotxyz, R.rt, tdts, tdtss);
                    QString toplain;
                    toplain.sprintf("G%02d %lf %lf %lf %lf %lf %lf %.6e %.6e", R.name[i], xyz[0], xyz[1], xyz[2],
                        dotxyz[0], dotxyz[1], dotxyz[2], tdts, tdtss);
                    solveresultshow->appendPlainText(toplain);
                }
            }
            else if (mode == 2)//GPS单系统单点定位与测速 
            {
                SppResult Result = {};

                int num = SPPpos(R, GPS, ion, Result, Fitmodelnum);
                double blh[3] = {}; xyztoblh(Result.xyzt[0], Result.xyzt[1], Result.xyzt[2], blh);
                QString toplain;
                toplain.sprintf("%lf %lf %lf %.6e %lf %lf %lf %.6e %lf %lf %lf %d",
                    Result.xyzt[0], Result.xyzt[1], Result.xyzt[2],
                    Result.xyzt[3], Result.xyztspeed[0], Result.xyztspeed[1],
                    Result.xyztspeed[2], Result.xyztspeed[3], blh[0], blh[1],
                    blh[2], num);
                solveresultshow->appendPlainText(toplain);
            }
        }
    }
    fclose(fp);
}

void UBSerials::RINEX_BDS_sat(const char* SAT_filename, const char* OBS_filename, int mode) {

    FILE* fp;
    char str[100] = {};
    eph_bds2 BDS[66] = {}; double ion[24][8] = {};
    read_BDS_EPH_RINEX(SAT_filename, BDS, ion);
    //星历构建完成，下面构建观测值


    fp = fopen(OBS_filename, "r");
    while (!feof(fp)) {
        fgets(str, 100, fp);
        if (strstr(str, "END OF HEADER"))
            break;
    }
    while (!feof(fp)) {
        fgets(str, 100, fp);
        if (strstr(str, ">"))
        {
            GPSOBS R = {};
            COMMONTIME ct = {}; int epoch_sat_num = 0, no = 0;
            sscanf(str, ">%d %d %d %d %d %lf %d %d", &ct.Year, &ct.Month, &ct.Day, &ct.Hour, &ct.Minute, &ct.Second, &no, &epoch_sat_num);
            R.rt = com2unixtime(ct);//获取观测历元时间
            QString toplain_ct;
            toplain_ct.sprintf("GPSTime:%4d.%02d.%02d %02d:%02d:%lf", ct.Year, ct.Month, ct.Day, ct.Hour, ct.Minute, ct.Second);
            solveresultshow->appendPlainText(toplain_ct);
            if (no != 0)
                continue;//观测值无效
            char rrstr[400] = {}; int k = 0;
            //在一个历元的所有观测值中获取BDS系统B1信号观测值
            for (int j = 0; j < epoch_sat_num; j++) {
                fgets(rrstr, 400, fp);
                if (strstr(rrstr, "C"))//读到BDS观测值
                {
                    char substr[100] = {};
                    memcpy(substr, rrstr, 3);
                    sscanf(substr, "C%d", &R.name[k]);
                    memset(substr, '\0', sizeof(substr));
                    memcpy(substr, rrstr + 3, 14);
                    sscanf(substr, "%lf", &R.R0[k]);
                    memset(substr, '\0', sizeof(substr));
                    memcpy(substr, rrstr + 17, 16);
                    sscanf(substr, "%lf", &R.adr[k]);
                    memset(substr, '\0', sizeof(substr));
                    memcpy(substr, rrstr + 35, 14);
                    sscanf(substr, "%f", &R.dopp[k]);
                    memset(substr, '\0', sizeof(substr));
                    memcpy(substr, rrstr + 49, 16);
                    sscanf(substr, "%f", &R.Cno[k]);
                    if (R.R0[k] || R.adr[k] || R.dopp[k])//有观测值则取值
                        k++;
                }

            }
            //观测值读取完毕，以下开始计算卫星位置
            R.num = k;//该历元BDS单系统观测值数量

            if (mode == 1)//计算卫星位置
            {
                double xyz[3] = {}, dotxyz[3] = {}, tdts = 0.0, tdtss = 0.0;
                for (int i = 0; i < R.num; i++) {
                    BDSsatpos(BDS[R.name[i]], 'C', R.name[i], R.R0[i], xyz, dotxyz, R.rt, tdts, tdtss);
                    QString toplain;
                    toplain.sprintf("C%02d %lf %lf %lf %lf %lf %lf %.6e %.6e", R.name[i], xyz[0], xyz[1], xyz[2],
                        dotxyz[0], dotxyz[1], dotxyz[2], tdts, tdtss);
                    solveresultshow->appendPlainText(toplain);
                }
            }

            else if (mode == 2)//BDS单系统北斗定位 
            {

                SppResult Result;

                int choice = 0;
                COMMONTIME rct = time2epoch(R.rt);
                choice = rct.Hour;
                int num = SPPpos(R, BDS, ion[choice], Result, Fitmodelnum);
                double blh[3] = {}; xyztoblh(Result.xyzt[0], Result.xyzt[1], Result.xyzt[2], blh);
                QString toplain;
                toplain.sprintf("%lf %lf %lf %.6e %lf %lf %lf %.6e %lf %lf %lf %d",
                    Result.xyzt[0], Result.xyzt[1], Result.xyzt[2],
                    Result.xyzt[3], Result.xyztspeed[0], Result.xyztspeed[1],
                    Result.xyztspeed[2], Result.xyztspeed[3], blh[0], blh[1],
                    blh[2], num);
                solveresultshow->appendPlainText(toplain);
            }

        }
    }
    fclose(fp);
}

void UBSerials::BINEARY_sat(const char* file, int mode) {
    FILE* fp;//文件指针
    int maxlen = 0;//文件长度    
    fp = fopen(file, "rb");
    while (!feof(fp))
    {
        char str[100];
        fread(str, 1, 1, fp);
        maxlen++;
    }
    printf("读取的文件中一共有%d字节的数据\n", maxlen);
    fclose(fp);//第一次遍历，记录文件字节数

    //模拟串口数据的缓冲区
    USARTbuff buff = {};//缓冲区
    unsigned char message[2 * 8192];//数据区(上次缓冲区剩余＋本次缓冲区)

    //开始模拟串口传输
    int i = 0; USARTbuff left = {};
    //定义接收机启动后的全局变量
    eph_t eph[36] = {};//GPS星历
    eph_bds2 bds[66] = {};//北斗星历
    double ion[8] = {}, ion_bds[8] = {};//GPS、BDS电离层延迟改正参数结构体
    double ion_OEM7[8] = {};
    int slen = 0; int history = 0;//单次数距长度slen，累计数据长度history

    fp = fopen(file, "rb");

    while (!feof(fp)) {
        //固定模拟的单次长度为4096
        slen = 4096;//模拟长度
        i++; history += slen;
        if (history <= maxlen)
            fread(buff.buff, 1, slen, fp);
        else
        {
            fread(buff.buff, 1, maxlen - (history - slen), fp);
            slen = maxlen - (history - slen);
        }
        buff.bufflen = slen;


        //构建有效消息序列数据区(上次缓冲区剩余＋本次缓冲区)
        int messagex = 0;
        {
            memcpy(message, left.buff, left.bufflen);
            memcpy(message + left.bufflen, buff.buff, buff.bufflen);
            messagex = buff.bufflen + left.bufflen;
        }


        //从有效数据区中获取消息报告
        breport epoch[400] = {};
        int epochnum = 0;
        epochnum = getbinaryreport(epoch, message, messagex);
        memset(left.buff, 0x00, sizeof(left.buff)); left.bufflen = 0;//上一次剩余数据区清零
        left.bufflen = messagex - epoch[epochnum].start;//重新构建剩余数据区结构(定长)
        memcpy(left.buff, message + epoch[epochnum].start, left.bufflen);//重新构建剩余数据区

        if (epochnum == 0)    continue;

        //定位主循环
        for (int j = 0; j < epochnum; j++) {
            GPSOBS R = {};//观测值结构体

            //北斗二代星历
            if (epoch[j].ID == 1047) {
                int prn;
                prn = getbds2eph(bds, message, epoch[j]);
                bds[prn].statu = EPHYES;
            }


            //GPS星历
            if (epoch[j].ID == 7) {
                int prn;
                prn = getsat(eph, message, epoch[j]);
                eph[prn].statu = EPHYES;
            }
            //电离层，因为老师提供的文件有两种板卡协议，0927数据一定要加上"OEM7"参数调用重载，Com_16不需要
            //以下 为GPS电离层参数
            if (epoch[j].ID == 8) {
                getion(message, ion, epoch[j]);
                if (ion[0] < 1e-100)
                    getion(message, ion, epoch[j], "OEM7");//扩展支持OEM7板卡协议
            }
            //以下为BDS电离层参数，需调用重载
            if (epoch[j].ID == 2010) {
                getion(message, ion_bds, epoch[j], "BDS");
            }
            //如果读到观测值，则进行定位尝试
            if (epoch[j].ID == 631 || epoch[j].ID == 43) {
                SppResult Result = {};
                getobs(R, message, epoch[j]);
                QString toplain_ct;
                COMMONTIME ct = time2epoch(R.rt);
                toplain_ct.sprintf("GPSTime:%4d.%02d.%02d %02d:%02d:%lf", ct.Year, ct.Month, ct.Day, ct.Hour, ct.Minute, ct.Second);
                solveresultshow->appendPlainText(toplain_ct);
                if (mode == 1) {//GPS卫星位置
                    for (int i = 0; i < R.num; i++) {
                        double xyz[3] = {}, dxyz[3] = {}, dts = 0.0, ddts = 0.0;
                        if (eph[R.name[i]].statu == EPHYES)
                        {
                            getsatelliteposition(eph[R.name[i]], 'G', R.name[i], R.R0[i], xyz, dxyz, R.rt, dts, ddts);
                            QString toplain;
                            toplain.sprintf("G%02d %lf %lf %lf %lf %lf %lf %.6e %.6e", R.name[i], xyz[0], xyz[1], xyz[2],
                                dxyz[0], dxyz[1], dxyz[2], dts, ddts);
                            solveresultshow->appendPlainText(toplain);
                        }
                    }
                }
                else if (mode == 2) {//GPS测站位置
                    int num = SPPpos(R, eph, ion, Result, Fitmodelnum);
                    double blh[3] = {}; xyztoblh(Result.xyzt[0], Result.xyzt[1], Result.xyzt[2], blh);
                    QString toplain;
                    toplain.sprintf("%lf %lf %lf %.6e %lf %lf %lf %.6e %lf %lf %lf %d",
                        Result.xyzt[0], Result.xyzt[1], Result.xyzt[2],
                        Result.xyzt[3], Result.xyztspeed[0], Result.xyztspeed[1],
                        Result.xyztspeed[2], Result.xyztspeed[3], blh[0], blh[1],
                        blh[2], num);
                    solveresultshow->appendPlainText(toplain);
                }
                else if (mode == 3) {//BDS卫星位置
                    for (int i = 0; i < R.num; i++) {
                        double xyz[3] = {}, dxyz[3] = {}, dts = 0.0, ddts = 0.0;
                        if (bds[R.name[i]].statu == EPHYES)
                        {
                            BDSsatpos(bds[R.name[i]], 'C', R.name[i], R.R0[i], xyz, dxyz, R.rt, dts, ddts);
                            QString toplain;
                            toplain.sprintf("C%02d %lf %lf %lf %lf %lf %lf %.6e %.6e", R.name[i], xyz[0], xyz[1], xyz[2],
                                dxyz[0], dxyz[1], dxyz[2], dts, ddts);
                            solveresultshow->appendPlainText(toplain);
                        }
                    }
                }
                else if (mode == 4) {//BDS测站位置
                    int num = SPPpos(R, bds, ion, Result, Fitmodelnum);
                    double blh[3] = {}; xyztoblh(Result.xyzt[0], Result.xyzt[1], Result.xyzt[2], blh);
                    QString toplain;
                    toplain.sprintf("%lf %lf %lf %.6e %lf %lf %lf %.6e %lf %lf %lf %d",
                        Result.xyzt[0], Result.xyzt[1], Result.xyzt[2],
                        Result.xyzt[3], Result.xyztspeed[0], Result.xyztspeed[1],
                        Result.xyztspeed[2], Result.xyztspeed[3], blh[0], blh[1],
                        blh[2], num);
                    solveresultshow->appendPlainText(toplain);
                }

            }
            //输出接收机原始结果
            if (epoch[j].ID == 47) {
                if (mode == 2 || mode == 4) {
                    int start = epoch[j].start;
                    double lat = bit2double(message + start + 28 + 8);
                    double lon = bit2double(message + start + 28 + 16);
                    double he = bit2double(message + start + 28 + 24) + bit2float(message + start + 28 + 32);
                    //注意接收机直接输出的是海拔高，需要与其内置的水准面模型相加才能得到直接解算的椭球高he
                    QString toplain;
                    toplain.sprintf("Reciver solution: WGS-84 %lf %lf %lf", lat, lon, he);
                    solveresultshow->appendPlainText(toplain);
                }
            }
        }

        if (history > maxlen)
            break;//超出文件字节数
    }
    fclose(fp);//关闭文件

}

#pragma 帮助程序子线程
void UBSerials::starthelper(void) {
    QPushButton* Helperbt = new QPushButton("启用帮助文档程序", this);
    Helperbt->setFixedSize(300*wb, 50*hb);
    Helperbt->move(300*wb, 1140*hb);
    QLabel* helperlabel = new QLabel("查看帮助请点击===>",this);
    QFont font; font.setBold(true); font.setPixelSize(25 * wb);
    helperlabel->setFont(font);
    helperlabel->setFixedSize(280*wb, 50*hb);
    helperlabel->move(50*wb, 1140*hb);

    connect(Helperbt, &QPushButton::clicked, this ,&UBSerials::HelperInit);
}
void UBSerials::HelperInit(void) {
    Helpercontainer = new QWidget();
    Helpercontainer->setWindowTitle("帮助系统");
    Helpercontainer->setContentsMargins(5, 5, 5, 5);
    Helper = new QWebEngineView(Helpercontainer);
    
    Helper->setMinimumSize(1200*wb, 800*hb);
    Helper->move(20*wb, 30*hb);
    Helper->setZoomFactor(2.0 * wb);

    QHBoxLayout* hlayout = new QHBoxLayout(Helpercontainer);
    hlayout->setSpacing(10);
    hlayout->setContentsMargins(5, 5, 5, 5);
    hlayout->addWidget(Helper);

    QNetworkProxyFactory::setUseSystemConfiguration(false);
    QString htmlFilePath = QApplication::applicationDirPath() + "/helperresource/main_helper.html";
    Helper->page()->load(QUrl(htmlFilePath));
    Helper->show();
   
    Helpercontainer->show();
}

void UBSerials::btn_label_fontresize(void) {
    QList<QPushButton*>btnlist = this->findChildren<QPushButton*>();
    for (QPushButton* btn : btnlist) {
        QFont font;
        font.setPixelSize(round(24 * wb));
        btn->setFont(font);
    }
    QList<QLabel*>lblist = this->findChildren<QLabel*>();
    for (QLabel* lb : lblist) {
        QFont font;
        font.setPixelSize(round(24 * wb));
        if (lb->text() == "查看帮助请点击===>") 
            continue;
        lb->setFont(font);
    }
    QList<QComboBox*>cboxlist = this->findChildren<QComboBox*>();
    for (QComboBox* cbox : cboxlist) {
        QFont font;
        font.setPixelSize(round(24 * wb));
        cbox->setFont(font);
    }
    QList<QPlainTextEdit*>ptelist = this->findChildren<QPlainTextEdit*>();
    for (QPlainTextEdit* pte : ptelist) {
        QFont font;
        font.setPixelSize(round(24 * wb));
        pte->setFont(font);
    }
    
}

#pragma 主线程构造函数
UBSerials::UBSerials(QWidget *parent)
    : QMainWindow(parent)
{
    QScreen* screen = QGuiApplication::primaryScreen();
    int w = screen->size().width();
    int h = screen->size().height();
    wb = w / 2520.0;//横比例因子 
    hb = h / 1680.0;//纵比例因子
    
    this->setFixedSize(2020 * wb, 1200 * hb);
    this->setWindowTitle("UBSerials");

    RecesiveAreaInit();
    computingshowAreaInit();
    SendAreaInit();
    SerialSetupInit();
    mapshowInit();
    StartUSART();
    dateLCDInit();
    File_solveInit();
    starthelper();
    btn_label_fontresize();
    portnum_TimeID = this->startTimer(1000);
    painter_TimeID = this->startTimer(1000);

}

#pragma 主线程析构函数
UBSerials::~UBSerials()
{}
