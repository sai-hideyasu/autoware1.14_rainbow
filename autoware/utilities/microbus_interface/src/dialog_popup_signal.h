#ifndef DIALOG_popup_signal_H
#define DIALOG_popup_signal_H

#include <QDialog>
#include <autoware_msgs/InterfacePopupSignal.h>

namespace Ui {
class Dialog_popup_signal;
}

class Dialog_popup_signal : public QDialog
{
	Q_OBJECT

public:
    explicit Dialog_popup_signal(bool use_accel_intervention, QWidget *parent = 0);
    ~Dialog_popup_signal();

	void setDin(const bool din0, const bool din1, const bool din2, const bool din3,
		const bool accel_intervention);//canからのボタン指令を受け取る
	void setPopupSignal(const uint8_t signal, const std::string str, const bool on_acc_intervention);//メインウィンドウからsignalIDと対応する表示文字列をセット
	void endFrom(const bool ok);//表示を終了してメインウィンドウにsignalIDに対してOKかキャンセルを通達する
	void blinkerCessation();//ウィンカー左、右のsignalIDがある場合にこの関数が呼ばれた場合、このウィンドウを閉じて

	bool getProcessFlag() const;
	void setWaypointID(const uint32_t id);
protected:
	void showEvent(QShowEvent *e);
	void timerEvent(QTimerEvent * e);
private:
    Ui::Dialog_popup_signal *ui;

	const int TIMER_START_DIN = 1000 / 30;//canからの入力を確認するタイマーのミリ秒数
	const int TIMER_END = 1000;

	bool use_accel_intervention_;//アクセル介入を許可するか？(コンストラクタでの指定)
	bool on_accel_intervention_;//アクセル介入を使用するか？(setPopupSignalでの指定)
	bool din0_, din1_, din2_, din3_;//can501のボタントリガー
	bool accel_intervention_;//can501のアクセル介入フラグ
	uint8_t signal_;
	bool process_flag_;
	uint32_t way_id_;

	int din_timer_id_;//canボタンを処理するタイマーID
	int end_timer_id_;//終了処理用タイマーID
private slots:
	void click_end();
	void click_cancel();
};

#endif // DIALOG_popup_signal_H
