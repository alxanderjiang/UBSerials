#pragma execution_character_set("utf-8")//����������
#include <QMainWindow>
#include<QApplication>
#include<QLabel>
#include<QPlainTextEdit>
#include<QPushButton>
#include<QmessageBox>
#include<QCombobox>
#include<QScreen>

class Stage : public QMainWindow
{
    Q_OBJECT

public:
    Stage(QWidget *parent = nullptr);
    ~Stage();
    
    void TimetransInit(void);//ʱ��ת����������ʼ��
    void Refreshtimeshow(time_t time, double seconds);//ʱ��ת�������ʾ����

    void CdTransInit(void);//����ת����������ʼ��
    
    void Trp_ionInit(void);//���������㹦������ʼ��

    void Stage_resize(void);//��Ļ��С�任

private:
    QPushButton* Timetransbt;
    QPushButton* Timetransclearbt;
    
    QPlainTextEdit* GPSTweek;
    QPlainTextEdit* GPSTseconds;

    QPlainTextEdit* UnixTtimes;
    QPlainTextEdit* UnixTseconds;

    QPlainTextEdit* Yearedit;
    QPlainTextEdit* Monthedit;
    QPlainTextEdit* Dayedit;
    QPlainTextEdit* Houredit;
    QPlainTextEdit* Minuteedit;
    QPlainTextEdit* Secondsedit;

    QPlainTextEdit* XYZedit;
    QPlainTextEdit* BLHedit;

    QPushButton* Cdtransbt;
    QPushButton* Cdtransclearbt;

    QPlainTextEdit* RR;
    QPlainTextEdit* SP;
    QPlainTextEdit* IonEdit_a;
    QPlainTextEdit* IonEdit_b;
    QComboBox* Trapmodelselect;
    
    
};
