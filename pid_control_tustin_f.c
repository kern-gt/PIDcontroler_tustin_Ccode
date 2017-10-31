/**~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
PID制御モジュール　Ver1.xx Alpha
pid_control_tustin_f.c

機能：パラメータ設定/取得、偏差PID,微分先行形P-ID、比例微分先行形I-PD
	全てfloat型変数を使用

作成日：2017/10/31

作成者：kern-gt

備考：float型専用
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~**/

/*************************************************************************************
	
	PID制御アルゴリズムについて
	以下の機能を実装している。

	・速度型（増分型）アルゴリズム
	・比例・微分項に１次遅れ型不完全微分を採用
	・双一次変換により伝達関数を離散化
	・自動整合制御型リセットワインドアップ制御


	//パラメータ調整法************************************************
	#ジーグラー・ニコルスの最適調整法について
	この調整法には2つあり、
	・ステップ応答法
	・限界感度法(省略)
	がある。（ステップ応答法の変形に速度応答法があるがここでは触れない。）
	ステップ応答法が良く用いられるため、表を載せておく。

		ステップ応答法によるPID制御構造体の最適調整法
		T:制御対象の等価時定数	K:等価ゲイン　L:等価むだ時間
	
		制御動作	比例ゲインKp	積分時間Ti	微分時間Td	評価指標（目標とする応答）
		P　		T/(KL)								減衰比25%
		PI 		0.9T/(KL)	3.3L					減衰比25%
		PID 	1.2T/(KL)	2.0L		0.5L		減衰比25%

	#CHR法について
	等価時定数Tに比べ、等価むだ時間Lが小さい領域(L/T=0.125~1)で適用が可能。
	外乱抑制特性と目標値追従特性のどちらかを最適するものに分けることができる。
	さらに応答を行き過ぎなしと行き過ぎ(=オーバーシュート)20％に分けた4通りの場合の
	最適PID制御構造体に設定が可能である。行き過ぎの場合は早く収束する。

		CHR法
		ゲインの決定法に外乱抑制最適化と目標値追従最適化の2種がある。
		この二つはトレードオフの関係であるのでどちらかを選ぶ。

		1.外乱抑制最適
		T:制御対象の等価時定数	K:等価ゲイン　L:等価むだ時間
	
		制御動作	比例ゲインKp	積分時間Ti	微分時間Td	評価指標（目標とする応答）
		P　		0.3T/(KL)							行き過ぎなし
		PI 		0.6T/(KL)	4L						行き過ぎなし
		PID 	0.95T/(KL)	2.4L		0.4L 		行き過ぎなし

		P　		0.7T/(KL)							行き過ぎ20%
		PI 		0.7T/(KL)	2.3L					行き過ぎ20%
		PID 	1.2T/(KL)	2L			0.42L		行き過ぎ20%

		2.目標値追従最適
		T:制御対象の等価時定数	K:等価ゲイン　L:等価むだ時間
	
		制御動作	比例ゲインKp	積分時間Ti	微分時間Td	評価指標（目標とする応答）
		P　		0.3T/(KL)							行き過ぎなし
		PI 		0.35T/(KL)	1.2L					行き過ぎなし
		PID 	0.6T/(KL)	1L			0.5L 		行き過ぎなし

		P　		0.7T/(KL)							行き過ぎ20%
		PI 		0.6T/(KL)	1L						行き過ぎ20%
		PID 	0.95T/(KL)	1.35L		0.47L		行き過ぎ20%


	************************************************************


**************************************************************************/

#include "pid_control_tustin_f.h"

/**~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
＊　関数名　：InitPid
＊　機能　　：PID制御構造体初期化
＊　引数　　：*pid_state_p=PID制御構造体, dt=制御周期[s]
＊　戻り値　：
＊　備考　　：
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~**/
void InitPid(PIDParameter_t *pid_state_p)
{
	(*pid_state_p).pgain = 1.0f;
	(*pid_state_p).ti    = 0.1f; 
	(*pid_state_p).td    = 0.1f;
	(*pid_state_p).tf_p  = 0.1f;
	(*pid_state_p).tf_d  = 0.1f;
	(*pid_state_p).t     = 1.0f;

	(*pid_state_p).outmax =  100.0f;
	(*pid_state_p).outmin = -100.0f;
	(*pid_state_p).delta_outmax =  1.0f;
	(*pid_state_p).delta_outmin = -1.0f;

	(*pid_state_p).error[0] = 0.0f;
	(*pid_state_p).error[1] = 0.0f;
	(*pid_state_p).error[2] = 0.0f;

	(*pid_state_p).feedbackbuf[0] = 0.0f;
	(*pid_state_p).feedbackbuf[1] = 0.0f;
	(*pid_state_p).feedbackbuf[2] = 0.0f;

	(*pid_state_p).velocity_p_buf[0] = 0.0f;
	(*pid_state_p).velocity_p_buf[1] = 0.0f;
	(*pid_state_p).velocity_i_buf[1] = 0.0f;
	(*pid_state_p).velocity_d_buf[1] = 0.0f;
	(*pid_state_p).velocity_d_buf[1] = 0.0f;
	(*pid_state_p).velocity_d_buf[1] = 0.0f;
    (*pid_state_p).velocity_buf[0]   = 0.0f;
    (*pid_state_p).velocity_buf[1]   = 0.0f;

	(*pid_state_p).pidout[0] = 0.0f;
	(*pid_state_p).pidout[1] = 0.0f;

	(*pid_state_p).limited_pidout[0] = 0.0f;
	(*pid_state_p).limited_pidout[1] = 0.0f;

	(*pid_state_p).reset_windup_feedback = 0.0f;
	(*pid_state_p).reset_windup_gain = 0.0f;
}

#if 0
/**~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
＊　関数名　：SetPidDt
＊　機能　　：制御周期設定関数
＊　引数　　：*pid_state_p=PID制御構造体, dt=制御周期[s]
＊　戻り値　：
＊　備考　　：
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~**/
void SetPidDt(PIDParameter_t *pid_state_p, float dt)
{
	(*pid_state_p).t = dt;
}


/**~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
＊　関数名　：GetPidDt
＊　機能　　：制御周期取得関数
＊　引数　　：*pid_state_p=PID制御構造体, :dt=制御周期
＊　戻り値　：
＊　備考　　：
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~**/
void GetPidDt(PIDParameter_t *pid_state_p, float *dt)
{
	*dt = (*pid_state_p).t;
}


/**~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
＊　関数名　：SetPidDff
＊　機能　　：微分係数設定関数
＊　引数　　：*pid_state_p=PID制御構造体, dff=微分係数(0.1~0.125)
＊　戻り値　：
＊　備考　　：
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~**/
void SetPidDff(PIDParameter_t *pid_state_p, float dff)
{
	(*pid_state_p).dff = dff;
}


/**~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
＊　関数名　：GetPidDff
＊　機能　　：微分係数取得関数
＊　引数　　：*pid_state_p=PID制御構造体, *dff=微分係数
＊　戻り値　：
＊　備考　　：
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~**/
void GetPidDff(PIDParameter_t *pid_state_p, float *dff)
{
	*dff = ((*pid_state_p).dff);
}


/**~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
＊　関数名　：SetPidGain
＊　機能　　：PIDゲイン設定関数
＊　引数　　：*pid_state_p=PID制御構造体, pgain=比例ゲイン, ti=積分時間, td=微分時間
＊　戻り値　：
＊　備考　　：
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~**/
void SetPidGain(PIDParameter_t *pid_state_p, float pgain, float ti, float td)
{
	(*pid_state_p).pgain = pgain;
	(*pid_state_p).ti 	 = ti;
	(*pid_state_p).td 	 = td;
}


/**~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
＊　関数名　：GetPidGain
＊　機能　　：PIDゲイン取得関数
＊　引数　　：*pid_state_p=PID制御構造体, *pgain=比例ゲイン, *ti=積分時間,
　　　　　　　　*td=微分時間
＊　戻り値　：
＊　備考　　：
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~**/
void GetPidGain( PIDParameter_t  *pid_state_p, float *pgain, float *ti,	float *td)
{
	*pgain = (*pid_state_p).pgain;
	*ti    = (*pid_state_p).ti;
	*td    = (*pid_state_p).td;
}


/**~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
＊　関数名　：SetPidOutlim
＊　機能　　：操作量限界値設定関数
＊　引数　　：*pid_state_p=PID制御構造体, outmax=操作量上限値, outmin=操作量下限値
＊　戻り値　：
＊　備考　　：
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~**/
void SetPidOutlim(PIDParameter_t *pid_state_p, float outmax, float outmin)
{
	(*pid_state_p).outmax = outmax;
	(*pid_state_p).outmin = outmin;
}


/**~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
＊　関数名　：GetPidOutlim
＊　機能　　：操作量限界値取得関数
＊　引数　　：*pid_state_p=PID制御構造体, *outmax=操作量最大値, *outmin=操作量最小値
＊　戻り値　：
＊　備考　　：
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~**/
void GetPidOutlim(PIDParameter_t *pid_state_p, float *outmax, float *outmin)
{
	*outmax = (*pid_state_p).outmax;
	*outmin = (*pid_state_p).outmin;
}


/**~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
＊　関数名　：
＊　機能　　：操作変化量限界値設定関数
＊　引数　　：*pid_state_p=PID制御構造体, delta_outmax=操作増加量限界値,
　　　　　　　　delta_outmin=操作減少量限界値
＊　戻り値　：
＊　備考　　：
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~**/
void SetPidDeltaoutlim(PIDParameter_t *pid_state_p, float delta_outmax, float delta_outmin)
{
	(*pid_state_p).delta_outmax = delta_outmax;
	(*pid_state_p).delta_outmin = delta_outmin;
}


/**~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
＊　関数名　：GetPidDeltaoutlim
＊　機能　　：操作変化量限界値取得関数
＊　引数　　：*pid_state_p=PID制御構造体, *delta_outmax=操作変化量限界値,
　　　　　　　　*delta_outmin=操作変化量最小値
＊　戻り値　：
＊　備考　　：
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~**/
void GetPidDeltaoutlim(PIDParameter_t *pid_state_p,	float *delta_outmax,　float *delta_outmin)
{
	*delta_outmax = (*pid_state_p).delta_outmax;
	*delta_outmin = (*pid_state_p).delta_outmin;
}
#endif

/**~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
＊　関数名　：VResPID
＊　機能　　：PID制御
＊　引数　　：*pid_state_p=PID制御構造体, setvalue=目標値, feedback_value=フィードバック信号入力
＊　戻り値　：操作量
＊　備考　　：速度型アルゴリズム＋不完全微分＋リセットワインドアップ制御
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~**/
float VResPID(PIDParameter_t *pid_state_p, float setvalue, float feedback_value)
{
	float tmp = 0.0f;
	float tmp2= 0.0f;
	float velocity_pid = 0.0;						//速度PID項出力

	//偏差
	(*pid_state_p).error[0] = setvalue - feedback_value;

	//速度比例演算
	tmp = 2.0f*(*pid_state_p).pgain*((*pid_state_p).error[0] - (*pid_state_p).error[1]);
	tmp2= ((*pid_state_p).t - 2.0f*(*pid_state_p).tf_p)*(*pid_state_p).velocity_p_buf[1];
	(*pid_state_p).velocity_p_buf[0] = (tmp -tmp2)/((*pid_state_p).t + 2.0f*(*pid_state_p).tf_p);
	(*pid_state_p).velocity_p_buf[1] = (*pid_state_p).velocity_p_buf[0];

	//速度積分演算
	tmp = (*pid_state_p).error[0] - (*pid_state_p).reset_windup_feedback;
	(*pid_state_p).velocity_i_buf[0] = (*pid_state_p).pgain * tmp / (*pid_state_p).ti;

	//速度微分演算
	tmp = (*pid_state_p).error[0] - 2.0f*(*pid_state_p).error[1] + (*pid_state_p).error[2];
	tmp = 4.0f*(*pid_state_p).pgain*(*pid_state_p).td * tmp / ((*pid_state_p).t*((*pid_state_p).t+2.0f*(*pid_state_p).tf_d));
	tmp = tmp - 2.0f*(*pid_state_p).t*(*pid_state_p).velocity_d_buf[1];
	(*pid_state_p).velocity_d_buf[0] = tmp - ((*pid_state_p).t-2.0f*(*pid_state_p).tf_d)*(*pid_state_p).velocity_d_buf[2];

	(*pid_state_p).velocity_d_buf[2] = (*pid_state_p).velocity_d_buf[1];
	(*pid_state_p).velocity_d_buf[1] = (*pid_state_p).velocity_d_buf[0];

	//偏差をバッファに保存
	(*pid_state_p).error[2] = (*pid_state_p).error[1];
	(*pid_state_p).error[1] = (*pid_state_p).error[0];

	//操作量積算
	(*pid_state_p).velocity_buf[0] = (*pid_state_p).velocity_p_buf[0] + (*pid_state_p).velocity_i_buf[0] + (*pid_state_p).velocity_d_buf[0];
    //(*pid_state_p).velocity_buf[0] = (*pid_state_p).velocity_p_buf[0] + (*pid_state_p).velocity_d_buf[0];

	//速度PID操作量を積分
	(*pid_state_p).pidout[0] = ((*pid_state_p).t*((*pid_state_p).velocity_buf[0] + (*pid_state_p).velocity_buf[1])/2.0f) + (*pid_state_p).pidout[1];
    (*pid_state_p).velocity_buf[1] = (*pid_state_p).velocity_buf[0];
    (*pid_state_p).pidout[1] = (*pid_state_p).pidout[0];

	//操作量上下限リミッタ
	if((*pid_state_p).pidout[0] > (*pid_state_p).outmax)
	{
		(*pid_state_p).limited_pidout[0] = (*pid_state_p).outmax;
	}else if((*pid_state_p).pidout[0] < (*pid_state_p).outmin)
	{
		(*pid_state_p).limited_pidout[0] = (*pid_state_p).outmin;
	}else{
		(*pid_state_p).limited_pidout[0] = (*pid_state_p).pidout[0];
	}

	//操作量変化率リミッタ
	if (((*pid_state_p).limited_pidout[0] - (*pid_state_p).limited_pidout[1]) > (*pid_state_p).delta_outmax)
	{
		(*pid_state_p).limited_pidout[0] = (*pid_state_p).limited_pidout[1] + (*pid_state_p).delta_outmax;
	}else if (((*pid_state_p).limited_pidout[0] - (*pid_state_p).limited_pidout[1]) < (*pid_state_p).delta_outmin)
	{
		(*pid_state_p).limited_pidout[0] = (*pid_state_p).limited_pidout[1] + (*pid_state_p).delta_outmin;
	}

	(*pid_state_p).limited_pidout[1] = (*pid_state_p).limited_pidout[0];

	(*pid_state_p).reset_windup_feedback = (*pid_state_p).reset_windup_gain*((*pid_state_p).pidout[0] - (*pid_state_p).limited_pidout[0]);

	return ((*pid_state_p).limited_pidout[0]);
}


/**~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
＊　関数名　：VResPI_D
＊　機能　　：微分先行形PID制御(PI-D)
＊　引数　　：*pid_state_p=PID制御構造体, setvalue=目標値, feedback_value=フィードバック信号入力
＊　戻り値　：操作量
＊　備考　　：速度型アルゴリズム＋不完全微分＋リセットワインドアップ制御
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~**/
#if 0
float VResPI_D(PIDParameter_t *pid_state_p,　float setvalue,	float feedback_value)
{
	
	return (0.0f);
}
#endif

/**~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
＊　関数名　：VResI_PD
＊　機能　　：比例微分先行形PID制御(I-PD)
＊　引数　　：*pid_state_p=PID制御構造体,  setvalue=目標値, feedback_value=フィードバック信号入力
＊　戻り値　：操作量
＊　備考　　：速度型アルゴリズム＋不完全微分＋リセットワインドアップ制御
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~**/
#if 0
float VResI_PD(PIDParameter_t *pid_state_p,　float setvalue,	float feedback_value)
{
	return (0.0f);
}
#endif