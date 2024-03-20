#include "UBSerials.h"
#include"satpos.h"
#pragma ȫ�ֱ���
SppResult Result = {};//��λ����ṹ��
UARTbuff Left = {};//ʣ��������
UARTbuff buff = {};//������
unsigned char message[2 * 8192] = {};//������(�ϴλ�����ʣ�࣫���λ�����)
//������ջ��������ȫ�ֱ���
eph_t eph[36] = {};//GPS����
eph_bds2 bds[65] = {};//BDS����
double ion[8] = {}; double ion_bds[8];//������ӳٸ��������ṹ��
GPSOBS R = {};//�۲�ֵ�ṹ��
gtime_t tobsnow = {};//��ǰGPSʱ��
long bdt_utc = 0;//��ǰBDS��UTC����������BDS��������

FILE* fp, *fo, *fs;//����ԭʼ�������ļ�������ļ�ָ��


#pragma ��������ʼ��
void UBSerials::RecesiveAreaInit(void) {

    //��������ʼ��
    receiveArea = new QPlainTextEdit(this);
    receiveArea->setFixedSize(800 * wb, 400 * hb);
    receiveArea->move(320*wb, 20*hb);
    receiveArea->setReadOnly(true);

    //��ս�������ť����ۺ���
    QPushButton* clearResArea = new QPushButton("��ս�����", this);
    clearResArea->setFixedSize(150 * wb, 50 * hb);
    clearResArea->move(320*wb, 430*hb);

    connect(clearResArea, &QPushButton::clicked, [&]() {
        receiveArea->clear();
        });
    //��������ǩ
    QLabel* resArealabel = new QLabel("�������ݽ�������ԭʼ���ݽ������棩", this);
    resArealabel->setFixedSize(400 * wb, 50 * hb);
    resArealabel->move(520*wb, 430*hb);

}

#pragma �Խ�������ʾ�����ʼ��
void UBSerials::computingshowAreaInit(void) {
    //�Խ�������ʾ����ʼ��
    computingshowArea = new QPlainTextEdit(this);
    computingshowArea->setFixedSize(400 * wb, 400 * hb);
    computingshowArea->move(200*wb, 730*hb);
    computingshowArea->setReadOnly(true);
    
    //�Խ�����ʾ����ǩ
    QLabel* Systemselectlabel = new QLabel("����ϵͳ", this);//ϵͳѡ����ǩ
    Systemselect = new QComboBox(this);//ϵͳѡ���
    QLabel* Saveselectlabel = new QLabel("�ļ�����", this);//����������ģʽѡ���ǩ
    Saveselect = new QComboBox(this);//����������ģʽѡ���
    QLabel* Fitmodellabel = new QLabel("��Ȩģ��", this);//��Ȩģ��ѡ���ǩ
    Fitmodel = new QComboBox(this);//��Ȩģ��ѡ���

    Systemselect->addItem("GPS");
    Systemselect->addItem("BDS");
    
    Saveselect->addItem("������");
    Saveselect->addItem("����");

    Fitmodel->addItem("��λ��");
    Fitmodel->addItem("�߶Ƚ�1");
    Fitmodel->addItem("�߶Ƚ�2");
    Fitmodel->addItem("α���׼��");
    Fitmodel->addItem("L1�����");
    Fitmodel->addItem("�����");

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

    //��ʼ���Խ��㰴ť
    realtimesolveButton = new QPushButton("�����Խ���", this);
    realtimesolveButton->setFixedSize(150 * wb, 50 * hb);
    realtimesolveButton->move(30*wb, 1020*hb);
    realtimesolveButton->setDisabled(false);//һ��ʼֻ�ܴ��Խ���

    connect(realtimesolveButton, &QPushButton::clicked, [&]() {
        if (serialPort==NULL||!serialPort->isOpen()) {
            QMessageBox::critical(this, "���ڴ�ʧ��", "��ȷ�ϴ����Ƿ���ȷ����");
        }
        else {
            this->realtimesolveflag = 1;//�����Խ��㹦�ܱ�־
            shutsolveButton->setDisabled(false);//shutʹ��
            realtimesolveButton->setDisabled(true);//set�ر�

            //�������ļ�����·���ļ��Ի���
            if (Saveselect->currentText() == "����") {
                QString currentpath = QDir::currentPath() + "/Example/";//��ǰ·��
                QString dlgtitle = "ѡ��������ļ��ͽ���ļ�����·��";//�ļ��Ի�������
                QString filter = "�����ļ�(*.*)";//�ļ���ʾ����
                QString folderName = QFileDialog::getExistingDirectory(this, dlgtitle, currentpath);

                QDateTime current_date_time = QDateTime::currentDateTime();
                QString date_time = current_date_time.toString("yyyyMMddhhmm");
                QString ofpath = folderName + "/" + date_time + "_OriginalOBS.txt";
                QString orpath = folderName + "/" + date_time + "_SolvedResult.txt";
                char t1[200] = {}, t2[200] = {};
                fp = fopen(ofpath.toLocal8Bit().data(), "wb");//ԭʼ���ݱ���
                fs = fopen(orpath.toLocal8Bit().data(), "w");
                fprintf(fs, "Sys GPST X Y Z dt B L H vX vY vZ vdt\n");
                Saveselect->setDisabled(true);
            }
            
            Serial_CMD_System();//һ��׼��������д�봮������
        }
        });
    
    //��ʼ���Խ���ֹͣ��ť
    shutsolveButton = new QPushButton("�ر��Խ���", this);
    shutsolveButton->setFixedSize(150 * wb, 50 * hb);
    shutsolveButton->move(30*wb, 1080*hb);
    shutsolveButton->setDisabled(true);//һ��ʼ�޷���ͣ�Խ���

    connect(shutsolveButton, &QPushButton::clicked, [&]() {
        this->realtimesolveflag = 0;//�����Խ��㹦�ܱ�־
        realtimesolveButton->setDisabled(false);
        shutsolveButton->setDisabled(true);

        serialPort->write("unlog\r\n");

        if (Saveselect->currentText() == "����") {
            fclose(fp);
            fclose(fs);
            Saveselect->setDisabled(false);
        }
        });

}

#pragma ���ڷ�������ʼ��
void UBSerials::SendAreaInit(void) {
    //��������ʼ��
    sendArea = new QPlainTextEdit(this);
    sendArea->setFixedSize(800 * wb, 100 * hb);
    sendArea->move(320*wb, 500*hb);

    //���Ͱ�ť��ʼ������ۺ���
    sendButton = new QPushButton("����", this);
    sendButton->setFixedSize(150 * wb, 50 * hb);
    sendButton->move(320*wb, 630*hb);
    sendButton->setDisabled(true);

    connect(sendButton, &QPushButton::clicked, [&]() {
        QString data = sendArea->toPlainText();
        data.append("\r\n");//�Զ������������Ļ��з��ͻس���
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

    //��շ�������ť����ۺ���
    QPushButton* clearsendArea = new QPushButton("��շ�����", this);
    clearsendArea->setFixedSize(150 * wb, 50 * hb);
    clearsendArea->move(480*wb, 630*hb);

    connect(clearsendArea, &QPushButton::clicked, [&]() {
        sendArea->clear();
        });
}

#pragma �������ó�ʼ��
void UBSerials::SerialSetupInit(void) {
    //�½�combbox����
    this->portnum = new QComboBox(this);
    this->botrate = new QComboBox(this);
    this->datasize = new QComboBox(this);
    this->checksize = new QComboBox(this);
    this->stopsize = new QComboBox(this);
    this->sendMode = new QComboBox(this);
    this->resMode = new QComboBox(this);


    //Ϊÿ��combox���ѡ��
    this->botrate->addItem("115200");
    this->botrate->addItem("9600");
    this->botrate->addItem("19200");
    this->botrate->addItem("38400");
    this->botrate->addItem("57600");//�μ�QT���ֺ�UB482�û��ֲ���ò�����

    this->datasize->addItem("8");//ͬ�ϣ���������λ

    this->stopsize->addItem("1");//ͬ�ϣ�����ֹͣλ

    this->checksize->addItem("��У��");
    this->checksize->addItem("��У��");
    this->checksize->addItem("żУ��");

    this->resMode->addItem("ASCII");//�ı�ָ�����
    this->resMode->addItem("HEX");//������ָ�����

    this->sendMode->addItem("ASCII");
    this->sendMode->addItem("HEX");


    //Ϊÿ��combox���ñ�ǩ
    QLabel* portnumlabel = new QLabel("���ڱ��", this);
    QLabel* botratelabel = new QLabel("������", this);
    QLabel* datasizelabel = new QLabel("����λ", this);
    QLabel* stopsizelabel = new QLabel("ֹͣλ", this);
    QLabel* checksizelabel = new QLabel("У��λ", this);
    QLabel* resmodelabel = new QLabel("����ģʽ", this);
    QLabel* sendmodelabel = new QLabel("����ģʽ", this);

    //����combox��lable�����б�
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

    //ѭ����������
    for (int i = 0; i < setups.size(); i++) {
        setups[i]->setFixedSize(150 * wb, 50 * hb);
        setups[i]->move(150*wb, (20 + i * 80)*hb);
        labels[i]->setFixedSize(100 * wb, 50 * hb);
        labels[i]->move(30 * wb, (25 + i * 80) * hb);
    }
}

#pragma ���ô�������
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

#pragma ���ô����б�/��Ȩģ�͸��£�������1HZ��
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
    if (Fitmodel->currentText() == "��λ��")
        Fitmodelnum = 0;
    else if (Fitmodel->currentText() == "�߶Ƚ�1")
        Fitmodelnum = 1;
    else if (Fitmodel->currentText() == "�߶Ƚ�2")
        Fitmodelnum = 2;
    else if (Fitmodel->currentText() == "α���׼��")
        Fitmodelnum = 3;
    else if (Fitmodel->currentText() == "L1�����")
        Fitmodelnum = 4;
    else if (Fitmodel->currentText() == "�����")
        Fitmodelnum = 5;
}
void UBSerials::timerEvent(QTimerEvent* e) {
    if(e->timerId()==portnum_TimeID)
    {
        Refreshports();//���´����б�
        Refreshfitmodel();//���¶�Ȩģ��
    }
    if (e->timerId() == painter_TimeID) {
        repaint();
    }
}

#pragma �������������ݻ�ȡ
void UBSerials::StartUSART(void) {

    //��ʼ�����򿪴��ڡ��͡��رմ��ڡ���ť����
    startUSART = new QPushButton("�򿪴���", this);
    endUSART = new QPushButton("�رմ���", this);
    startUSART->setFixedSize(270 * wb, 50 * hb);
    endUSART->setFixedSize(270 * wb, 50 * hb);
    startUSART->move(30*wb, 570*hb);
    endUSART->move(30*wb, 630*hb);

    //�Ͽ����Ӳۺ���
    endUSART->setDisabled(true);

    connect(endUSART, &QPushButton::clicked, [&]() {
        //���͡�������ťʧЧ�����Ӱ�ť��λ
        sendButton->setDisabled(true);
        startUSART->setDisabled(false);
        endUSART->setDisabled(true);
        //��������
        serialPort->close();
        });

    //�����Ӳۺ���
    connect(startUSART, &QPushButton::clicked, [&]() {
        //���Ӱ�ťʧЧ�����������Ͱ�ť��λ
        startUSART->setDisabled(true);
        endUSART->setDisabled(false);
        sendButton->setDisabled(false);
        //���Ӳ���
        if (portnum->currentText() != "")
            USART();//���Ӵ��ں���
        else
            QMessageBox::critical(this, "���ڴ�ʧ��", "��ȷ�ϴ����Ƿ���ȷ����");
        });

}
void UBSerials::USART(void) {
    QSerialPort::BaudRate Baud;//������
    QSerialPort::DataBits Data;//����λ
    QSerialPort::StopBits Stop;//ֹͣλ
    QSerialPort::Parity check;//У��

    //���ò�����
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

    //����λ����
    if (this->datasize->currentText() == "8") Data = QSerialPort::Data8;

    //ֹͣλ����
    if (this->stopsize->currentText() == "1") Stop = QSerialPort::OneStop;
    
    //У��λ����
    if (this->checksize->currentText() == "��У��") check = QSerialPort::NoParity;
    else if (this->checksize->currentText() == "��У��") check = QSerialPort::OddParity;
    else if (this->checksize->currentText() == "żУ��") check = QSerialPort::EvenParity;

    //���ڶ����ʼ��������
    serialPort = new QSerialPort(this);
    serialPort->setBaudRate(Baud);
    serialPort->setDataBits(Data);
    serialPort->setStopBits(Stop);
    serialPort->setParity(check);
    serialPort->setPortName(this->portnum->currentText());//���ں�
    serialPort->setReadBufferSize(BUFFSIZE);
    
    //���ô��ڶ���Ϊ�ɶ���д
    if (serialPort->open(QSerialPort::ReadWrite)) {
        connect(serialPort, &QSerialPort::readyRead, [&]() {
            slen = serialPort->bytesAvailable();//�������ݳ���
            
            //ԭʼ���ݻ�ȡ
            auto data = serialPort->readAll();
            
            //���ڽ�����������ģʽ��ʾ
            if (resMode->currentText() == "HEX") {      //�ֽ�ģʽ
                QString hex = data.toHex(' ');
                receiveArea->appendPlainText(hex);
            }
            else {                                          //�ı�ģʽ
                QString str = QString(data);
                receiveArea->appendPlainText(str);
            }
            
            //�Խ���
            memset(buffer, 0, 8192);//���������
            for (int i = 0; i < slen; i++)
            {
                buffer[i] = unsigned char(data[i]);
                buff.buff[i] = buffer[i];
            }
            
            if (realtimesolveflag == 1 && Saveselect->currentText() == "����")
                fwrite(buffer, 1, slen, fp);
            
            if (realtimesolveflag == 1) {
                int messagex=RefreshUsedmessage(message);
                computing(message, messagex);//�Խ����߳�
            }
           
            
            });
    }else {
        QMessageBox::critical(this, "���ڴ�ʧ��", "��ȷ�ϴ����Ƿ���ȷ����");
        sendButton->setDisabled(true);
        startUSART->setDisabled(false);
        endUSART->setDisabled(true);
    };
}

#pragma �Խ����̺߳���
int UBSerials::RefreshUsedmessage(unsigned char* message) {
    
    buff.bufflen = slen;
    //������Ч������
    int messagex = 0;
    
    memset(message, 0x00, sizeof(message));//����һ��ʹ�õ���Ч���������
    memcpy(message, Left.buff, Left.bufflen);//������һ��ʣ�������
    memcpy(message + Left.bufflen, buff.buff, buff.bufflen);//���Ʊ�������������
    messagex = buff.bufflen + Left.bufflen;//���������ݳ���
    
    
    //����Ч�������л�ȡ��Ϣ����
    breport epoch[400] = {};
    int epochnum = 0;
    epochnum = getbinaryreport(epoch, message, messagex);
    memset(Left.buff, 0x00, sizeof(Left.buff)); Left.bufflen = 0;//��һ��ʣ������������
    Left.bufflen = messagex - epoch[epochnum].start;//���¹���ʣ���������ṹ(����)
    memcpy(Left.buff, message + epoch[epochnum].start, Left.bufflen);//���¹���ʣ��������

    return messagex;//����Ч������������Ϊ����ֵ����
}
void UBSerials::computing(unsigned char* message, int messagex) {
    
    //��ȡ��������Ϣ����
    breport epoch[400];
    int epochnum = 0;
    epochnum = getbinaryreport(epoch, message, messagex);

    //��λ��ѭ��
    for (int j = 0; j < epochnum; j++) {

        //GPS����
        if (epoch[j].ID == 7) {
            int prn;
            prn = getsat(eph, message, epoch[j]);
            eph[prn].statu = EPHYES;
        }
        //����B1�ź�����
        if (epoch[j].ID == 1047) {
            int prn;
            prn = getbds2eph(bds, message, epoch[j]);
            bds[prn].statu = EPHYES;
        }
        //GPS����㣬��Ϊ��ʦ�ṩ���ļ������ְ忨Э�飬0927����һ��Ҫ����"OEM7"�����������أ�Com_16����Ҫ
        if (epoch[j].ID == 8) {
            getion(message, ion, epoch[j]);
        }
        //����ΪBDS�������������������
        if (epoch[j].ID == 2010) {
            getion(message, ion_bds, epoch[j], "BDS");
        }
        //����ΪBDS��UTC֮�������������ʱ����
        if (epoch[j].ID == 2012) {
            bdt_utc = bit2long(message + epoch[j].start + 24 + 32);
        }
        //��������۲�ֵ������ж�λ���ԣ�ID=631ΪOEM7Э�飬ID=43ΪUB482Э��
        if (epoch[j].ID == 631 || epoch[j].ID == 43) {
            
           
            getobs(R, message, epoch[j]);//��ȡ�۲�ֵ
            
            //GPS��ϵͳ���㶨λ
            if (tobsnow.time != R.rt.time && Systemselect->currentText()=="GPS") {
                int flag = SPPpos(R, eph, ion, Result, Fitmodelnum);
                tobsnow.time = R.rt.time; tobsnow.second = R.rt.second;//����ϵͳʱ��
                sprintf(Result.sysytem, "GPS");
                if (flag >= 4) {
                    char str[200] = {}; double blh[3] = {};
                    xyztoblh(Result.xyzt[0], Result.xyzt[1], Result.xyzt[2], blh);
                    double Vspeed = (Result.xyztspeed[0] * Result.xyztspeed[0])
                        + (Result.xyztspeed[0] * Result.xyztspeed[0])
                        + (Result.xyztspeed[0] * Result.xyztspeed[0]);
                    Vspeed = sqrt(Vspeed);
                    sprintf(str, "From GPS:\n Lat:%lf\n Lon:%lf\n Hight:%lf\n Speed:%lf\n GPST:\%lld", blh[0], blh[1], blh[2], Vspeed, Result.tobs.time);
                    if (Saveselect->currentText() == "����")
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
            //GPS��ϵͳ���㶨λ����
            
            //BDS��ϵͳ���㶨λ
            if (tobsnow.time != R.rt.time && Systemselect->currentText()=="BDS") {
                int flag_bds = SPPpos(R, bds, ion_bds, Result, Fitmodelnum);
                tobsnow.time = R.rt.time; tobsnow.second = R.rt.second;//����ϵͳʱ��
                sprintf(Result.sysytem, "BDS");
                if (flag_bds >= 4) {
                    char str[200] = {}; double blh[3] = {};
                    xyztoblh(Result.xyzt[0], Result.xyzt[1], Result.xyzt[2], blh);
                    double Vspeed = (Result.xyztspeed[0] * Result.xyztspeed[0])
                        + (Result.xyztspeed[0] * Result.xyztspeed[0])
                        + (Result.xyztspeed[0] * Result.xyztspeed[0]);
                    Vspeed = sqrt(Vspeed);
                    sprintf(str, "From BDS:\n Lat:%lf\n Lon:%lf\n Hight:%lf\n Speed:%lf\n GPST:\%lld", blh[0], blh[1], blh[2], Vspeed, Result.tobs.time);
                    if (Saveselect->currentText() == "����")
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
            //BDS��ϵͳ��λ����
        }
        
        //������ջ�ԭʼ���
        if (epoch[j].ID == 47) {
            int start = epoch[j].start;
            double lat = bit2double(message + start + 28 + 8);
            double lon = bit2double(message + start + 28 + 16);
            double he = bit2double(message + start + 28 + 24) + bit2float(message + start + 28 + 32);
            //ע����ջ�ֱ��������Ǻ��θߣ���Ҫ�������õ�ˮ׼��ģ����Ӳ��ܵõ�ֱ�ӽ���������he
            gtime_t ubobstime = {};
            ubobstime = gpst2time(int(breport2GPStime(epoch[j], message).Week), breport2GPStime(epoch[j], message).Second);
            //�Խ������Ԫʱ��
            char str[200] = {}; sprintf(str, "From UB482 %lf %lf %lf\n", lat, lon, he);
            if (ubobstime.time == tobsnow.time)
                this->computingshowArea->appendPlainText(QString(str));
        }
    }
}

#pragma �ߵµ�ͼ��ʾ
void UBSerials::mapshowInit(void) {
    Gaodemapshow = new QWebEngineView(this);
    Gaodemapshow->setFixedSize(800 * wb, 800 * hb);
    Gaodemapshow->move(1150*wb, 20*hb);
    Gaodemapshow->setZoomFactor(1.5 * wb);

    QNetworkProxyFactory::setUseSystemConfiguration(false);
    Gaodemapshow->page()->load(QUrl(htmlFilePath));
    Gaodemapshow->show();

    QPushButton* mapchange = new QPushButton("��ͼ�л�", this);
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
    
    QPushButton* Refreshmap = new QPushButton("��ͼ����", this);
    Refreshmap->setFixedSize(150 * wb, 50 * hb);
    Refreshmap->move(1150*wb, 830*hb);

    connect(Refreshmap, &QPushButton::clicked, [&]() {
        if(htmlFilePath.contains("Gaode",Qt::CaseInsensitive))
            Gaodemapshow->page()->runJavaScript("repoint([116.397428, 39.90923]);");//�ߵµ�ͼ
        else
            Gaodemapshow->page()->runJavaScript("repoint([116.404, 39.915]);");//�ٶȵ�ͼ
        });
    
    Realtimemap = new QPushButton("ʵʱչʾ",this);
    Realtimemap->setFixedSize(150 * wb, 50 * hb);
    Realtimemap->move(1320*wb, 830*hb);
    Realtimemap->setDisabled(false);
    
    shutRealtimemap = new QPushButton("�ر�ʵʱչʾ", this);
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
            QMessageBox::critical(this, "���ڴ�ʧ��", "��ȷ�ϴ����Ƿ���ȷ����");
        }
        
        });

    connect(shutRealtimemap, &QPushButton::clicked, [&]() {
        map_TimeID = 0;//���ý��̼�ʱ�����
        killTimer(map_TimeID);
        shutRealtimemap->setDisabled(true);
        Realtimemap->setDisabled(false);
        });

    QPushButton* clearAllmakers = new QPushButton("������б�ǵ�", this);
    clearAllmakers->setFixedSize(250 * wb, 50 * hb);
    clearAllmakers->move(1700*wb, 830*hb);

    connect(clearAllmakers, &QPushButton::clicked, [&]() {
        Gaodemapshow->page()->runJavaScript("clearmap();");//������б��
        });

    QPushButton* Singlepoint = new QPushButton("������", this);
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
            QMessageBox::critical(this, "��γ�ȸ�ʽ����", "��������ȷ�ľ���(Lng)��γ��(Lat)"); 
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

#pragma ��ͼ����ʼ������ʾ
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
    *///��ʾЧ�����⣬��ʱ�رո߶Ƚ�ͼ��
    for(int i=0;i<5;i++)
        azelShow->drawEllipse(QPoint(875*wb, 930*hb),int(40*(i+1)*wb), int(40*(i+1)*wb));
    qDebug() << 40 * (1) * wb << int(40 * (1) * wb);
    //���ƿ��ӿ������Ƿ�λ�Ǹ߶Ƚ�ͼ
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

#pragma LCDʱ����ʾ
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
            utc.time -= 14;//����Ϊ����ʱ
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

#pragma �ļ�������
void UBSerials::File_solveInit(void) {
    OBSfilechoice = new QPlainTextEdit(this);//�۲�ֵ�ļ�ѡ��
    SATfilechoice = new QPlainTextEdit(this);//�����ļ�ѡ��
    QLabel* OBSClabel = new QLabel("�۲�ֵ�ļ�", this);
    QLabel* SATClabel = new QLabel("�����ļ�", this);
    QPushButton* obsselectbt = new QPushButton("...", this);
    QPushButton* satselectbt = new QPushButton("...", this);
    
    Fileselect = new QComboBox(this);
    QLabel* FSelectlabel = new QLabel("�ļ���ʽ", this);

    File_system_select = new QComboBox(this);
    QLabel* F_sys_Selectlabel = new QLabel("����ģʽ", this);
    File_system_select->addItem("����λ��");
    File_system_select->addItem("��վλ��");
    
    Fileselect->addItem("RINEX");
    Fileselect->addItem("������");

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
        QString currentpath = QDir::currentPath() + "/Example/";//��ǰ·��
        QString dlgtitle = "ѡ��۲��ļ�";//�ļ��Ի�������
        QString filter = "�����ļ�(*.*) ;; �ı��ļ�(*.txt)";//�ļ���ʾ����
        QString fileName = QFileDialog::getOpenFileName(this, dlgtitle, currentpath, filter);
        if (!fileName.isEmpty())
            OBSfilechoice->setPlainText(fileName);
        });
    
    connect(satselectbt, &QPushButton::clicked, [&]() {
        QString currentpath = QDir::currentPath() + "/Example/";//��ǰ·��
        QString dlgtitle = "ѡ�������ļ�";//�ļ��Ի�������
        QString filter = "�����ļ�(*.*) ;; �ı��ļ�(*.txt)";//�ļ���ʾ����
        QString fileName = QFileDialog::getOpenFileName(this, dlgtitle, currentpath, filter);
        if (!fileName.isEmpty())
            SATfilechoice->setPlainText(fileName);
        });

    QPushButton* Filesolvebt = new QPushButton("�ļ�����", this);
    Filesolvebt->setFixedSize(150*wb, 50*hb);
    Filesolvebt->move(1580*wb, 1080*hb);
    connect(Filesolvebt, &QPushButton::clicked, this ,&UBSerials::FilesolvepageInit);

    QPushButton* Stagesolvebt = new QPushButton("�м�ֵ����", this);
    Stagesolvebt->setFixedSize(200*wb, 50*hb);
    Stagesolvebt->move(1750*wb, 1080*hb);
    connect(Stagesolvebt, &QPushButton::clicked, [&]() {
        children.show();
        });
}
void UBSerials::FilesolvepageInit(void) {
    /*��������*/
    Filesolvepage = new QWidget();
    Filesolvepage->setWindowTitle("�ļ�����");
    Filesolvepage->setFixedSize(1600*wb,800*hb);
    solveresultshow = new QPlainTextEdit(Filesolvepage);
    solveresultshow->setReadOnly(true);
    solveresultshow->setFixedSize(1500*wb, 700*hb);
    solveresultshow->move(50*wb, 80*hb);
    
    /*���㺯��*/
    QLabel* showlabel_1 = new QLabel("������", Filesolvepage);
    showlabel_1->setFixedSize(1500*wb, 40*hb); showlabel_1->setAlignment(Qt::AlignCenter);
    QFont font; font.setPixelSize(40*wb); showlabel_1->setFont(font);
    showlabel_1->move(50*wb, 30*hb);
    
    replaybt = new QPushButton("�켣����", Filesolvepage);
    replaybt->setFixedSize(200 * wb, 50 * hb);
    replaybt->move(1350 * wb, 30 * hb);
    font.setPixelSize(24 * wb);
    replaybt->setFont(font);
    solveresultshow->setFont(font);
    Filesolvepage->show();
    
    int flag = File_SAT_POS();

    /*��������ʾ*/
    if (flag == 1)
    {
        showlabel_1->setText("RINEX�ļ�����GPS����λ�ý��");
    }
    else if (flag == 2)
    {
        showlabel_1->setText("RINEX�ļ�����BDS����λ�ý��");
    }
    else if (flag == 5) {
        showlabel_1->setText("RINEX�ļ�GPS��վ���㶨λ���ٽ��");
    }
    else if (flag == 6) {
        showlabel_1->setText("RINEX�ļ�BDS��վ���㶨λ���ٽ��");
    }
    else if (flag == 3) {
        showlabel_1->setText("�������ļ�����GPS����λ�ý��");
    }
    else if (flag == 4) {
        showlabel_1->setText("�������ļ�����BDS����λ�ý��");
    }
    else if (flag == 7) {
        showlabel_1->setText("�������ļ�GPS��ϵͳ���㶨λ���ٽ��");
    }
    else if (flag == 8) {
        showlabel_1->setText("�������ļ�BDS��ϵͳ���㶨λ���ٽ��");
    }
    else if (flag == 0) {
        QMessageBox::critical(this, "����ʧ��", "���������ļ��ͽ���ѡ�������");
        Filesolvepage->close();
        return;
    }
    /*�켣���Ʋۺ���*/
    if (flag == 1 || flag == 2 || flag == 3 || flag == 4) {
        delete replaybt;
    }

    connect(replaybt, &QPushButton::clicked, [&]() {
        QString result = solveresultshow->toPlainText();//��ȡ������
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
    //���ȼ���ļ�·���Ƿ���ȷ
    FILE* satf;
    if (!(satf = fopen(OBSfilechoice->toPlainText().toLocal8Bit().data(), "r"))) {
        QMessageBox::critical(this, "�۲�ֵ�ļ���ʧ��", "����۲�ֵ�ļ�·���Ƿ���ȷ");
        return 0;
    }
    fclose(satf);
    if (!(satf = fopen(SATfilechoice->toPlainText().toLocal8Bit().data(), "r"))) {
        QMessageBox::critical(this, "�����ļ���ʧ��", "���������ļ�·���Ƿ���ȷ");
        return 0;
    }
    fclose(satf); 
    
    if (File_system_select->currentText() == "����λ��") {

        if (Fileselect->currentText() == "RINEX")
        {
            if (Systemselect->currentText() == "GPS")
            {
                solveresultshow->appendPlainText("PRN X Y Z X_speed Y_speed Z_speed Dt Dt_speed");
                RINEX_GPS_sat(SATfilechoice->toPlainText().toLocal8Bit().data(), OBSfilechoice->toPlainText().toLocal8Bit().data());
                return 1;//RINEX_GPS����λ��
            }
            else if (Systemselect->currentText() == "BDS")
            {
                solveresultshow->appendPlainText("PRN X Y Z X_speed Y_speed Z_speed Dt Dt_speed");
                RINEX_BDS_sat(SATfilechoice->toPlainText().toLocal8Bit().data(), OBSfilechoice->toPlainText().toLocal8Bit().data());
                return 2;//RINEX_��������λ��
            }
            
        }
        if (Fileselect->currentText() == "������") {
            if (SATfilechoice->toPlainText() != OBSfilechoice->toPlainText())
                {
                    QMessageBox::critical(this, "�������ļ���ƥ��", "���ڹ۲�ֵ�ļ��������ļ�ѡ�����������ͬ���ļ�·��");
                    return 0;
                }
            
            if (Systemselect->currentText() == "GPS") {    
                solveresultshow->appendPlainText("PRN X Y Z X_speed Y_speed Z_speed Dt Dt_speed");
                BINEARY_sat(SATfilechoice->toPlainText().toLocal8Bit().data(), 1);
                return 3;//GPS����������λ��
            }
            else if (Systemselect->currentText() == "BDS") {
                solveresultshow->appendPlainText("PRN X Y Z X_speed Y_speed Z_speed Dt Dt_speed");
                BINEARY_sat(SATfilechoice->toPlainText().toLocal8Bit().data(), 3);
                return 4;//BDS����������λ��
            }
        };
    }
    else {
        if (Fileselect->currentText() == "RINEX")
        {
            if (Systemselect->currentText() == "GPS") {
                solveresultshow->appendPlainText("X Y Z  Clock_Dt X_speed Y_speed Z_speed Clock_Dt_speed B L H satsum");
                RINEX_GPS_sat(SATfilechoice->toPlainText().toLocal8Bit().data(), OBSfilechoice->toPlainText().toLocal8Bit().data(), 2);
                return 5;//RINEX_GPS���㶨λ�����
            }
            else if (Systemselect->currentText() == "BDS") {
                solveresultshow->appendPlainText("X Y Z  Clock_Dt X_speed Y_speed Z_speed Clock_Dt_speed B L H satsum");
                RINEX_BDS_sat(SATfilechoice->toPlainText().toLocal8Bit().data(), OBSfilechoice->toPlainText().toLocal8Bit().data(), 2);
                return 6;//RINEX�������㶨λ�����
            }
        }
        if (Fileselect->currentText() == "������") {
            if (SATfilechoice->toPlainText() != OBSfilechoice->toPlainText())
            {
                QMessageBox::critical(this, "�������ļ���ƥ��", "���ڹ۲�ֵ�ļ��������ļ�ѡ�����������ͬ���ļ�·��");
                return 0;
            }

            if (Systemselect->currentText() == "GPS") {
                solveresultshow->appendPlainText("X Y Z  Clock_Dt X_speed Y_speed Z_speed Clock_Dt_speed B L H satsum");
                BINEARY_sat(SATfilechoice->toPlainText().toLocal8Bit().data(), 2);
                return 7;//GPS�����Ʋ�վ���㶨λ����
            }
            else if (Systemselect->currentText() == "BDS") {
                solveresultshow->appendPlainText("X Y Z  Clock_Dt X_speed Y_speed Z_speed Clock_Dt_speed B L H satsum");
                BINEARY_sat(SATfilechoice->toPlainText().toLocal8Bit().data(), 4);
                return 8;//BDS�����Ƶ��㶨λ����
            }
        }
    }

}

/*����Ϊ��RINEX�ļ���ֱ�ӽ������λ�õĺ���*/

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

    fclose(fp);//�����ļ�������prn�Ŷ�ȡ��ϣ����濪ʼ��ȡ����

    eph_t GPS[36] = {};
    double data[32] = {}; int snum[9] = {};
    for (int i = 0; i < namecount; i++)
    {
        readmessagefile(name[i], SAT_filename, data, snum);
        GPS[name[i]] = getsate(data);
        GPS[name[i]].statu = EPHYES;
    }//����������ɣ����湹����������
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

    //��������������ɣ����湹���۲�ֵ
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
            R.rt = com2unixtime(ct);//��ȡ�۲���Ԫʱ��
            QString toplain_ct;
            toplain_ct.sprintf("GPSTime:%4d.%02d.%02d %02d:%02d:%lf", ct.Year, ct.Month, ct.Day, ct.Hour, ct.Minute, ct.Second);
            solveresultshow->appendPlainText(toplain_ct);
            if (no != 0)
                continue;//�۲�ֵ��Ч
            char rrstr[400] = {}; int k = 0;
            //��һ����Ԫ�����й۲�ֵ�л�ȡGPSϵͳL1�źŹ۲�ֵ
            for (int j = 0; j < epoch_sat_num; j++) {
                fgets(rrstr, 400, fp);
                if (strstr(rrstr, "G"))//����GPS�۲�ֵ
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
                    if (R.R0[k] || R.adr[k] || R.dopp[k])//�й۲�ֵ��ȡֵ
                        k++;
                }

            }
            //�۲�ֵ��ȡ��ϣ����¿�ʼ��������λ��
            R.num = k;//����ԪGPS��ϵͳ�۲�ֵ����

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
            else if (mode == 2)//GPS��ϵͳ���㶨λ����� 
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
    //����������ɣ����湹���۲�ֵ


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
            R.rt = com2unixtime(ct);//��ȡ�۲���Ԫʱ��
            QString toplain_ct;
            toplain_ct.sprintf("GPSTime:%4d.%02d.%02d %02d:%02d:%lf", ct.Year, ct.Month, ct.Day, ct.Hour, ct.Minute, ct.Second);
            solveresultshow->appendPlainText(toplain_ct);
            if (no != 0)
                continue;//�۲�ֵ��Ч
            char rrstr[400] = {}; int k = 0;
            //��һ����Ԫ�����й۲�ֵ�л�ȡBDSϵͳB1�źŹ۲�ֵ
            for (int j = 0; j < epoch_sat_num; j++) {
                fgets(rrstr, 400, fp);
                if (strstr(rrstr, "C"))//����BDS�۲�ֵ
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
                    if (R.R0[k] || R.adr[k] || R.dopp[k])//�й۲�ֵ��ȡֵ
                        k++;
                }

            }
            //�۲�ֵ��ȡ��ϣ����¿�ʼ��������λ��
            R.num = k;//����ԪBDS��ϵͳ�۲�ֵ����

            if (mode == 1)//��������λ��
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

            else if (mode == 2)//BDS��ϵͳ������λ 
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
    FILE* fp;//�ļ�ָ��
    int maxlen = 0;//�ļ�����    
    fp = fopen(file, "rb");
    while (!feof(fp))
    {
        char str[100];
        fread(str, 1, 1, fp);
        maxlen++;
    }
    printf("��ȡ���ļ���һ����%d�ֽڵ�����\n", maxlen);
    fclose(fp);//��һ�α�������¼�ļ��ֽ���

    //ģ�⴮�����ݵĻ�����
    USARTbuff buff = {};//������
    unsigned char message[2 * 8192];//������(�ϴλ�����ʣ�࣫���λ�����)

    //��ʼģ�⴮�ڴ���
    int i = 0; USARTbuff left = {};
    //������ջ��������ȫ�ֱ���
    eph_t eph[36] = {};//GPS����
    eph_bds2 bds[66] = {};//��������
    double ion[8] = {}, ion_bds[8] = {};//GPS��BDS������ӳٸ��������ṹ��
    double ion_OEM7[8] = {};
    int slen = 0; int history = 0;//�������೤��slen���ۼ����ݳ���history

    fp = fopen(file, "rb");

    while (!feof(fp)) {
        //�̶�ģ��ĵ��γ���Ϊ4096
        slen = 4096;//ģ�ⳤ��
        i++; history += slen;
        if (history <= maxlen)
            fread(buff.buff, 1, slen, fp);
        else
        {
            fread(buff.buff, 1, maxlen - (history - slen), fp);
            slen = maxlen - (history - slen);
        }
        buff.bufflen = slen;


        //������Ч��Ϣ����������(�ϴλ�����ʣ�࣫���λ�����)
        int messagex = 0;
        {
            memcpy(message, left.buff, left.bufflen);
            memcpy(message + left.bufflen, buff.buff, buff.bufflen);
            messagex = buff.bufflen + left.bufflen;
        }


        //����Ч�������л�ȡ��Ϣ����
        breport epoch[400] = {};
        int epochnum = 0;
        epochnum = getbinaryreport(epoch, message, messagex);
        memset(left.buff, 0x00, sizeof(left.buff)); left.bufflen = 0;//��һ��ʣ������������
        left.bufflen = messagex - epoch[epochnum].start;//���¹���ʣ���������ṹ(����)
        memcpy(left.buff, message + epoch[epochnum].start, left.bufflen);//���¹���ʣ��������

        if (epochnum == 0)    continue;

        //��λ��ѭ��
        for (int j = 0; j < epochnum; j++) {
            GPSOBS R = {};//�۲�ֵ�ṹ��

            //������������
            if (epoch[j].ID == 1047) {
                int prn;
                prn = getbds2eph(bds, message, epoch[j]);
                bds[prn].statu = EPHYES;
            }


            //GPS����
            if (epoch[j].ID == 7) {
                int prn;
                prn = getsat(eph, message, epoch[j]);
                eph[prn].statu = EPHYES;
            }
            //����㣬��Ϊ��ʦ�ṩ���ļ������ְ忨Э�飬0927����һ��Ҫ����"OEM7"�����������أ�Com_16����Ҫ
            //���� ΪGPS��������
            if (epoch[j].ID == 8) {
                getion(message, ion, epoch[j]);
                if (ion[0] < 1e-100)
                    getion(message, ion, epoch[j], "OEM7");//��չ֧��OEM7�忨Э��
            }
            //����ΪBDS�������������������
            if (epoch[j].ID == 2010) {
                getion(message, ion_bds, epoch[j], "BDS");
            }
            //��������۲�ֵ������ж�λ����
            if (epoch[j].ID == 631 || epoch[j].ID == 43) {
                SppResult Result = {};
                getobs(R, message, epoch[j]);
                QString toplain_ct;
                COMMONTIME ct = time2epoch(R.rt);
                toplain_ct.sprintf("GPSTime:%4d.%02d.%02d %02d:%02d:%lf", ct.Year, ct.Month, ct.Day, ct.Hour, ct.Minute, ct.Second);
                solveresultshow->appendPlainText(toplain_ct);
                if (mode == 1) {//GPS����λ��
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
                else if (mode == 2) {//GPS��վλ��
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
                else if (mode == 3) {//BDS����λ��
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
                else if (mode == 4) {//BDS��վλ��
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
            //������ջ�ԭʼ���
            if (epoch[j].ID == 47) {
                if (mode == 2 || mode == 4) {
                    int start = epoch[j].start;
                    double lat = bit2double(message + start + 28 + 8);
                    double lon = bit2double(message + start + 28 + 16);
                    double he = bit2double(message + start + 28 + 24) + bit2float(message + start + 28 + 32);
                    //ע����ջ�ֱ��������Ǻ��θߣ���Ҫ�������õ�ˮ׼��ģ����Ӳ��ܵõ�ֱ�ӽ���������he
                    QString toplain;
                    toplain.sprintf("Reciver solution: WGS-84 %lf %lf %lf", lat, lon, he);
                    solveresultshow->appendPlainText(toplain);
                }
            }
        }

        if (history > maxlen)
            break;//�����ļ��ֽ���
    }
    fclose(fp);//�ر��ļ�

}

#pragma �����������߳�
void UBSerials::starthelper(void) {
    QPushButton* Helperbt = new QPushButton("���ð����ĵ�����", this);
    Helperbt->setFixedSize(300*wb, 50*hb);
    Helperbt->move(300*wb, 1140*hb);
    QLabel* helperlabel = new QLabel("�鿴��������===>",this);
    QFont font; font.setBold(true); font.setPixelSize(25 * wb);
    helperlabel->setFont(font);
    helperlabel->setFixedSize(280*wb, 50*hb);
    helperlabel->move(50*wb, 1140*hb);

    connect(Helperbt, &QPushButton::clicked, this ,&UBSerials::HelperInit);
}
void UBSerials::HelperInit(void) {
    Helpercontainer = new QWidget();
    Helpercontainer->setWindowTitle("����ϵͳ");
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
        if (lb->text() == "�鿴��������===>") 
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

#pragma ���̹߳��캯��
UBSerials::UBSerials(QWidget *parent)
    : QMainWindow(parent)
{
    QScreen* screen = QGuiApplication::primaryScreen();
    int w = screen->size().width();
    int h = screen->size().height();
    wb = w / 2520.0;//��������� 
    hb = h / 1680.0;//�ݱ�������
    
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

#pragma ���߳���������
UBSerials::~UBSerials()
{}
