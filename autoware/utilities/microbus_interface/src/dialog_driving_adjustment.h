#ifndef DIALOG_DRIVING_ADJUSTMENT_H
#define DIALOG_DRIVING_ADJUSTMENT_H

#include <QDialog>

namespace Ui {
class Dialog_driving_adjustment;
}

class Dialog_driving_adjustment : public QDialog
{
	Q_OBJECT

public:
	explicit Dialog_driving_adjustment(QWidget *parent = 0);
	~Dialog_driving_adjustment();

	void valueClear();//初期状態にクリア
	int16_t getSubSteer();//現在のステア補正値を取得
	void setDin(const bool din0, const bool din1, const bool din2, const bool din3);//canからのボタン指令を受け取る
	void setWaypointCorrection(const int16_t base_corr, const int16_t sub_corr);//waypoint情報からsteer調整項目を取得する
protected:
	void showEvent(QShowEvent *e);
	void closeEvent(QCloseEvent *e);
	void timerEvent(QTimerEvent * e);
private:
	const uint16_t WAYPOINT_CORR_NO_READ = 10000;//waypoint情報を読み込まれてない場合はwaypoint_base_corr_とwaypoint_sub_corr_はこの値になる
	const int TIMER_START_DIN = 100;//canからの入力を確認するタイマーのミリ秒数
	const int TIMER_SHOW = 2000;//フォームを表示してから入力を受け付けるまでのタイマーのミリ秒数

	Ui::Dialog_driving_adjustment *ui;

	int16_t waypoint_base_corr_;//経路上のベース補正値
	int16_t waypoint_sub_corr_;//経路上のサブ補正値(このフォームで操作するもの)
	int16_t steer_plus_sub_;//現在の補正値
	bool din0_, din1_, din2_, din3_;//can501のボタントリガー
	int din_timer_id_;//canボタンを処理するタイマーID
	int show_timer_id_;//canからの起動直後は処理をさせないようにするためのタイマーID
	bool first_show_flag_;//フォーム表示後、少しの間trueとなる

	void leftArrow();
	void rightArrow();
	void endFrom(const bool per);
private slots:
	void click_end();
	void click_left();
	void click_right();
	void click_cancel();
};

#endif // DIALOG_DRIVING_ADJUSTMENT_H
