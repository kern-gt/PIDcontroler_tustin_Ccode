/**~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
PID制御ヘッダ　Ver1.xx Alpha
pid_control_tustin_f.h

機能：float型のPID制御関数
	伝達関数を双一次変換したのち、差分方程式を導出している。

作成日：2017/10/31

作成者：kern-gt

備考：float型専用

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~**/

#ifndef _PID_CONTROL_F_H_
#define _PID_CONTROL_F_H_

/**------------------------------------------------------------------------------
<<外部公開マクロ定義>>
------------------------------------------------------------------------------**/

//PID制御構造体初期化
/*初期化パラメータ------------------------------------------------------------
	pgain	= 1.0;			//比例ゲイン
	ti		= 0.1;			//積分時間
	td　		= 0.1;			//微分時間
	tf_p　	= 0.1;			//比例項不完全微分の微分係数(0.1~0.125)
	tf_d　	= 0.1;			//微分項不完全微分の微分係数(0.1~0.125)
	t　		= 1.0;			//サンプル制御周期　内部は(秒)

	outmax 	=  100.0;		//操作量最大値
	outmin 	= -100.0;		//操作量最小値
	delta_outmax =  1.0;	//操作変化量最大値
	delta_outmin = -1.0;	//操作変化量最小値

	error[3] 			= すべて0.0;	//偏差バッファ
	inputbuf[3] 		= すべて0.0;	//入力バッファ
	velocity_d 			= 0.0;		//速度D項出力
	pidout[3] 			= すべて0.0;	//位置PID項出力
 	limited_pidout[2]	= すべて0.0;	//制限済み位置PID出力
 	float reset_windup_feedback = 0.0;//リセットワインドアップフィードバック値
	reset_windup_gain 	= 0.1;		//リミッタ動作時に速度I項を無視する
-----------------------------------------------------------------------*/

/**----------------------------------------------------------------------------
<<外部公開型定義>>
-----------------------------------------------------------------------------**/

//PID制御構造体型
typedef struct
{
	//制御パラメータ
	float pgain;				//比例ゲイン
	float ti;					//積分時間
	float td;					//微分時間
	float tf_p;					//微分係数(0.1~0.125)
	float tf_d;					//微分係数(0.1~0.125)
	float t;					//制御周期(秒)

	//操作量リミッタ
	float outmax;				//操作量最大値
	float outmin;				//操作量最小値
	float delta_outmax;			//操作変化量最大値
	float delta_outmin;			//操作変化量最小値

	//PID内部変数
	float error[3];				//偏差バッファ
	float feedbackbuf[3];		//フィードバックバッファ
	float velocity_p_buf[2];    //
	float velocity_i_buf[1];
	float velocity_d_buf[3];
    float velocity_buf[2];
	float pidout[2];			//位置PID項出力
	float limited_pidout[2];	//制限済み位置PID出力
	float reset_windup_feedback;//リセットワインドアップフィードバック値
	float reset_windup_gain;	//リセットワインドアップゲイン

}PIDParameter_t;


/**-----------------------------------------------------------------------------
<<外部公開プロトタイプ定義>>
------------------------------------------------------------------------------**/
/**変数初期化--------------------------------------------------------------------**/
/*PID制御構造体初期化関数*/
extern void InitPid(PIDParameter_t *pid_state_p);	//PID制御構造体

/**パラメータ設定-------------------------------------------------------------------**/
/*制御周期設定関数　引数は(秒)*/
extern void SetPidDt(PIDParameter_t *pid_state_p,			//PID制御構造体
								float dt);			//制御周期

/*微分係数設定関数(0.1~0.125))*/
extern void SetPidDff(PIDParameter_t *pid_state_p,			//PID制御構造体
								float dff);			//微分係数

/*pidゲイン設定関数*/
extern void SetPidGain(PIDParameter_t *pid_state_p,			//PID制御構造体
				  				float pgain,		//比例ゲイン
				  				float ti,			//積分時間
				  				float td);			//微分時間

/*操作量限界値設定関数*/
extern void SetPidOutlim(PIDParameter_t *pid_state_p,		//PID制御構造体型
								float outmax,		//操作量上限値
								float outmin);		//操作量下限値

/*操作変化量限界値設定関数*/
extern void SetPidDeltaoutlim(PIDParameter_t *pid_state_p,	//PID制御構造体
					 		float delta_outmax,		//操作増加量限界値
					 		float delta_outmin);	//操作減少量限界値

/**パラメータ取得------------------------------------------------------------------**/
/*制御周期取得関数　戻り値は(秒)*/
extern void GetPidDt(PIDParameter_t *pid_state_p,			//PID制御構造体
								float *dt);			//制御周期

/*/微分係数取得関数*/
extern void GetPidDff(PIDParameter_t *pid_state_p,			//PID制御構造体
								float *dff);		//微分係数

/*pidゲイン読み出し関数*/
extern void GetPidGain(PIDParameter_t *pid_state_p,		//PID制御構造体
				  				float *pgain,		//比例ゲイン
				  				float *ti,			//積分時間
				  				float *td);			//微分時間

/*操作量限界値取得関数*/
extern void GetPidOutlim(PIDParameter_t *pid_state_p,		//PID制御構造体
								float *outmax,		//操作量最大値
								float *outmin);		//操作量最小値

/*操作変化量限界値取得関数*/
extern void GetPidDeltaoutlim(PIDParameter_t *pid_state_p,	//PID制御構造体
					  		float *delta_outmax,	//操作変化量最大値
					  		float *delta_outmin);	//操作変化量最小値

/**PID制御関数-------------------------------------------------------------------**/
/*実用偏差速度形ディジタルPID*/
extern float VResPID(PIDParameter_t *pid_state_p,	//PID制御構造体
					   			float setvalue,		//目標値
					   			float feedback_value);	//制御入力値

/*実用微分先行速度形ディジタルPID*/
extern float VResPI_D(PIDParameter_t *pid_state_p,	//PID制御構造体
					   	   		float setvalue,		//目標値
					   	   		float feedback_value);	//制御入力

/*実用比例微分先行速度形ディジタルPID*/
extern float VResI_PD(PIDParameter_t *pid_state_p,	//PID制御構造体
					   			float setvalue,		//目標値
								float feedback_value);	//制御入力


#endif /*_PID_CONTROL_F_H_*/
