//パネル表示の指令を保存するクラス
#ifndef PANEL_VIEW_FLAG
#define PANEL_VIEW_FLAG

#include <autoware_msgs/InterfacePopupSignal.h>

class PanelViewFlag
{
private:
	bool zidountenzissityu_;
	bool sakinidouzo_;
	bool saitamakogyodaigaku_;
	bool usetusimasu_;
	bool mamonakuaka_;
	bool taikosyatukamati_;
	bool horeisokudojunsyu_;

	uint8_t signal_;

	void clear()
	{
		zidountenzissityu_ = false;
		sakinidouzo_ = false;
		saitamakogyodaigaku_ = false;
		usetusimasu_ = false;
		mamonakuaka_ = false;
		taikosyatukamati_ = false;
		horeisokudojunsyu_ = false;

		signal_ = autoware_msgs::InterfacePopupSignal::SIGNAL_PANEL_NO;
	}
public:
	PanelViewFlag()
	{
		clear();
	}

	void set_zidountenzissityu(const bool flag)
	{
		zidountenzissityu_= flag;
	}
	void set_sakinidouzo(const bool flag)
	{
		sakinidouzo_ = flag;
	}
	void set_saitamakogyodaigaku(const bool flag)
	{
		saitamakogyodaigaku_ = flag;
	}
	void set_usetusimasu(const bool flag)
	{
		usetusimasu_ = flag;
	}
	void set_mamonakuaka(const bool flag)
	{
		mamonakuaka_ = flag;
	}
	void set_taikosyatukamati(const bool flag)
	{
		taikosyatukamati_ = flag;
	}
	void set_horeisokudojunsyu(const bool flag)
	{
		horeisokudojunsyu_ = flag;
	}

	void set_reset()
	{
		clear();
	}

	bool get_zidountenzissityu() const {return zidountenzissityu_;}
	bool get_sakinidouzo() const {return sakinidouzo_;}
	bool get_saitamakogyodaigaku() const {return saitamakogyodaigaku_;}
	bool get_usetusimasu() const {return usetusimasu_;}
	bool get_mamonakuaka() const {return mamonakuaka_;}
	bool get_taikosyatukamati() const {return taikosyatukamati_;}
	bool get_horeisokudojunsyu() const {return horeisokudojunsyu_;}

	uint8_t getViewSignal() const
	{
		if(usetusimasu_ == true)
			return autoware_msgs::InterfacePopupSignal::SIGNAL_PANEL_USETUSIMASU;
		else if(mamonakuaka_ == true)
			return autoware_msgs::InterfacePopupSignal::SIGNAL_PANEL_MAMONAKUAKA;
		else if(horeisokudojunsyu_ == true)
			return autoware_msgs::InterfacePopupSignal::SIGNAL_PANEL_HOREISOKUDOJUNSYU;
		else if(taikosyatukamati_ == true)
			return autoware_msgs::InterfacePopupSignal::SIGNAL_PANEL_TAIKOSYATUKAMATI;
		else if(sakinidouzo_ == true)
			return autoware_msgs::InterfacePopupSignal::SIGNAL_PANEL_SAKINIDOUZO;
		else if(zidountenzissityu_ == true)
			return autoware_msgs::InterfacePopupSignal::SIGNAL_PANEL_ZIDOUNTENZISSITYU;
		else if(saitamakogyodaigaku_ == true)
			return autoware_msgs::InterfacePopupSignal::SIGNAL_PANEL_SAITAMAKOGYODAIGAKU;

		return autoware_msgs::InterfacePopupSignal::SIGNAL_PANEL_NO;
	}
};

#endif