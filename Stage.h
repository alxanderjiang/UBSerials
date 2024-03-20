#pragma execution_character_set("utf-8")//防中文乱码
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
    
    void TimetransInit(void);//时间转换功能区初始化
    void Refreshtimeshow(time_t time, double seconds);//时间转换结果显示函数

    void CdTransInit(void);//坐标转换功能区初始化
    
    void Trp_ionInit(void);//电离层对流层功能区初始化

    void Stage_resize(void);//屏幕大小变换

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
