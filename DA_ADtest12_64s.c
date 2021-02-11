#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <termios.h>
#include <fcntl.h>
#include <errno.h>
#include "fbida.h"
#include "fbiad.h"
#define AD1_NUM	1	// ADボード番号
#define DA1_NUM	1	// DAボード番号

// バッファサイズ
#define DA_BUFF_CNT	0x4000	// DAデータの最大個数
#define AD_BUFF_CNT	0x1000	// ADデータの最大個数

// 励起光変調の最大値
#define DA_MOD_MAX	0x03333	// ThorLABS S1FC635 はカタログ上は0～5V, 5.5Vでクリッピングと書いてあるが、実際は1.0Vでクリップされているように見える

// １ライン分の構成タイミング

#define DA_SMPL_FREQ	54.0			// DAクロック周波数（時間設定の分解能を決める）
#define DA_SMPL_CNT	0x2000
#define DA_CNT_MEAS	20			// AD積分期間 (50Hz, 20ms)
#define DA_CNT_LOFF	2			// 励起光消し待ちDAクロック数
#define AD_SMPL_CNT	4096  	// 1箇所あたりのAD変換回数
#define AD_SMPL_FREQ	54.0			// AD変換周波数 = 50Hz × (1箇所あたりのAD変換回数)
/*typedef struct {
	char	*key;
	char	*name;
	char	*unit;
	float	fvalue;
	float	min;
	float	max;
	int	flag;
} param;
*/

typedef struct {
	char	*key;
	char	*name;
	char	*unit;
	float	fvalue;
	float	min;
	float	max;
	int	flag;
} param;

#define PARAMS_CNT 11

param	params[PARAMS_CNT] = { // keyは同じ文字列で始まる場合は文字列の長い順番で入れること
		{"f" , "Light frequency","Hz",16,   1, 200, 0},	// 0
		{"p" , "Light power"	,"%" ,  10,    0, 100, 0},	// 1
		{"xb", "X begin"	,"%" , -20, -100, 100, 0},	// 2
		{"yb", "Y begin"	,"%" , -20, -100, 100, 0},	// 3
		{"xe", "X end"	,"%" ,  20, -100, 100, 0},	// 4
		{"ye", "Y end"	,"%" ,  20, -100, 100, 0},	// 5
		{"xp", "X points"	,"%" ,  21,    1, 100, 0},	// 6
		{"yp", "Y points"	,"%" ,  21,    1, 100, 0},	// 7
		{"x" , "X position"	,"%" ,   0, -100, 100, 0},	// 8
		{"y" , "Y position"	,"%" ,   0, -100, 100, 0},	// 9
		{"w" , "Wait"		,"ms",  40,    0, 100, 0},	// 10
	};


ADBMSMPLREQ	AdBmSmplReq;
DASMPLREQ	DaSmplReq;
int		nRet;				// 関数戻り値

//unsigned short	DaSmplData[DA_BUFF_CNT][4];	// DAデータバッファ
int		nDaCount;			// DAデータバッファオーバーランチェック用変数

unsigned long	ADSmplData[AD_BUFF_CNT];	// ADデータバッファ
unsigned long	ADSmplNum;			// ADサンプル数

void CloseAndExit(int code, char *message)
{
	// DAがクローズする時にレンジが初期値に戻り、雑音が出力される
	// それを阻止するために先に外部リレーをOFFにする

  nRet = DaOutputDO(DA1_NUM, 0x00);	// リレーをOFFにする

  usleep(20000);			// リレーの接点がブレークするまで20ms待つ

	DaClose(DA1_NUM);
  AdClose(AD1_NUM);
	if(code != 0){
    		          fputs(message, stderr);
		              fprintf(stderr, "code = %x\n", code);
                }	exit(code);

}
void AdInit(void)
{
    // ADボードオープン
    nRet = AdOpen(AD1_NUM);
    if(nRet != AD_ERROR_SUCCESS) CloseAndExit(nRet, "Error: AdOpen()\n");

//	CN3の機能設定
//	nRet = AdSetFunction(AD1_NUM, AD_EX_DIO1, AD_CN_EXCLK_IN);	// 外部クロック用に端子を設定
	nRet = AdSetFunction(AD1_NUM, AD_EX_DIO1, AD_CN_EXTRG_IN);	// 外部トリガ用に端子を設定
	if(nRet != AD_ERROR_SUCCESS) CloseAndExit(nRet, "Error: AdSetFunction()\n");
/*
//	CN3の機能設定確認
	unsigned long ulFunction;
	nRet = AdGetFunction(AD1_NUM, AD_EX_DIO1, &ulFunction);
	if(nRet != AD_ERROR_SUCCESS) CloseAndExit(nRet, "Error: AdGetFunction()\n");
	fprintf(stderr, "EX_DIO1 = %ld\n", ulFunction);
*/
//	デジタルフィルタのオーバーサンプリングレートの設定
	nRet = AdSetFilter(AD1_NUM, AD_DF_256);
	if(nRet != AD_ERROR_SUCCESS) CloseAndExit(nRet, "Error: AdSetFilter()\n");
}

void AdSetup(void)
{
	// サンプリング条件の初期値読み込み
	nRet = AdBmGetSamplingConfig(AD1_NUM, &AdBmSmplReq);	// これやらないとカーネルパニックを起こす
	if(nRet != AD_ERROR_SUCCESS) CloseAndExit(nRet, "Error: AdGetSamplingConfig()\n");

	// サンプリング条件の設定
	AdBmSmplReq.ulChCount = 2;				// チャネル数は1
	AdBmSmplReq.SmplChReq[0].ulChNo = 1;		// ch1の
	AdBmSmplReq.SmplChReq[0].ulRange = AD_5V;		// 　レンジは±5V
	AdBmSmplReq.SmplChReq[1].ulChNo = 2;		// ch2の
	AdBmSmplReq.SmplChReq[1].ulRange = AD_5V;		// 　レンジは±5V
//	AdBmSmplReq.ulSamplingMode = AD_BM_SAMPLING;	// データはDMAでメモリへ転送（ADBMSMPLREQにはメンバが存在しない）
//	AdBmSmplReq.ulSingleDiff = AD_INPUT_SINGLE;	// 接続はシングルエンド（しかない）
	AdBmSmplReq.ulSmplNum = AD_SMPL_CNT;		// トリガ１回あたりの連続AD変換数
	AdBmSmplReq.ulSmplEventNum = 0;			// 完了サンプル数による通知は使わない
	AdBmSmplReq.ulSmplRepeat = 1;			// 繰り返し測定：測定点１ヶ所あたり明暗２回の測定をする
	AdBmSmplReq.ulBufferMode = AD_APPEND;		// 繰り返し測定：バッファは繰り返しごとに後ろへ追加していく
	AdBmSmplReq.fSmplFreq = AD_SMPL_FREQ;		// ADサンプリング周波数
	AdBmSmplReq.ulStartMode = AD_EXTTRG;		// 外部トリガでスタートする
	AdBmSmplReq.ulStopMode = AD_SMPLNUM;		// ulSmplNum完了でストップする
	AdBmSmplReq.ulStartTrigEdge = AD_UP_EDGE;		// トリガ立ち上がりでスタート
//	AdBmSmplReq.ulStopTrigEdge = AD_DOWN_EDGE;	// トリガ立ち下がりでストップ

	// サンプリング条件の設定
	nRet = AdBmSetSamplingConfig(AD1_NUM, &AdBmSmplReq);
	if(nRet != AD_ERROR_SUCCESS) CloseAndExit(nRet, "Error: AdSetSamplingConfig()\n");
}

void AdStart(void)
{
	// 連続サンプリングの開始
//	nRet = AdStartSampling(AD1_NUM, FLAG_ASYNC);
//	if(nRet != AD_ERROR_SUCCESS) CloseAndExit(nRet, "Error: AdStartSampling()\n");

	ADSmplNum = AD_SMPL_CNT  * 2;
	nRet = AdStartSamplingEx(AD1_NUM, &ADSmplData[0], ADSmplNum*sizeof(unsigned long));
	if(nRet != AD_ERROR_SUCCESS) CloseAndExit(nRet, "Error: AdStartSamplingEx()\n");
}

void AdGetData(void)
{
	unsigned long ulAdSmplStatus;
	unsigned long ulAdSmplCount;
	unsigned long ulAdAvailCount;

//	int flag = 1;
	while(1){
		nRet = AdGetStatus( 1, &ulAdSmplStatus, &ulAdSmplCount, &ulAdAvailCount );
		if(nRet != AD_ERROR_SUCCESS) CloseAndExit(nRet, "Error: AdGetStatus()\n");
    if(ulAdSmplStatus == AD_STATUS_STOP_SAMPLING) break;
		//	case AD_STATUS_WAIT_TRIGGER:
		//	case AD_STATUS_NOW_SAMPLING:
		//	fprintf(stderr, "Done: %ld, Remain: %ld\n", ulAdSmplCount, ulAdAvailCount);
		//	CloseAndExit(ulAdSmplStatus, "Error: AD busy.\n");
	}

	// データの取得
//	unsigned long ulSmplNum = AD_SMPL_CNT * iXPoints * 2;
//	nRet = AdGetSamplingData(AD1_NUM, &ADSmplData[0], &ulSmplNum);
//	if(nRet != AD_ERROR_SUCCESS) CloseAndExit(nRet, "Error: AdGetSamplingData()\n");
//	fprintf(stderr, "%lu, %lu\n", ADSmplNum, ulSmplNum);

	int n, m;
	double val1, val2;

	m = 0;
	for(n=0; n<AD_SMPL_CNT; n++){
		val1 = (ADSmplData[m++]/(double)0x0FFFFFF - (double)0.5) * (double)20.0;
    val2 = (ADSmplData[m++]/(double)0x0FFFFFF - (double)0.5) * (double)20.0;
		fprintf(stdout, "%d, %+le, %+le\n", n, val1, val2);
	}
}

void DaRelayON(void)
{
	// レンジ変更に伴う雑音が収まったころに外部リレーをONにする
	usleep(5000);				// 5ms待ってから
  nRet = DaOutputDO(DA1_NUM, 0x01);	// 外部リレーをONにする
  if(nRet != DA_ERROR_SUCCESS) CloseAndExit(nRet, "Error: DaOutputDO()\n");
	usleep(5000);				// リレーの接点がメークするまで5ms待つ
}

void GetArg(char *argv)
{
	int m, n, ret;

	for(m=0; m<PARAMS_CNT; m++)
  {
		for(n=0; argv[n]==params[m].key[n]; n++);
		if(params[m].key[n] == 0)
    {
			fputs(params[m].name, stdout);
			ret = sscanf(&argv[n], "%g", &params[m].fvalue);
			if(ret != 1)
      {
				fprintf(stdout, " 数値の読み込みに失敗しました '%s'\n", argv);
			} else {
				if(params[m].fvalue < params[m].min)
        {
					fprintf(stdout, " (指定された数値 %g は最小値 %g です)", params[m].fvalue, params[m].min);
					params[m].fvalue = params[m].min;
				}
				if(params[m].fvalue > params[m].max)
        {
					fprintf(stdout, " (指定された数値 %g は最大値 %g です)", params[m].fvalue, params[m].max);
					params[m].fvalue = params[m].max;
				}
				fprintf(stdout, " %g%s\n", params[m].fvalue, params[m].unit);
				params[m].flag = 1;
			}
			return;
		}
	}
	fprintf(stdout, "パラメータの指定方法に誤りがありそうです '%s'\n", argv);
}


//#define DA1_NUM	1
//#define DA_SMPL_CNT	0x2000
//#define DA_SMPL_FREQ	256.0
//int	nRet;

DASMPLREQ	DaSmplConfig;
unsigned short	DaSmplData[DA_SMPL_CNT][2];
int	nDaCount;
int i=0;
int j=0;
int k=0;
int l=0;


void DaInit(void)
	{
		// DAボードオープン
			nRet = DaOpen(DA1_NUM);
				if(nRet != DA_ERROR_SUCCESS) CloseAndExit(nRet, "Error: DaOpen()\n");
		// ボード設定
					nRet = DaSetBoardConfig(DA1_NUM, 0x10000, NULL, NULL, 0);
					if(nRet != DA_ERROR_SUCCESS) CloseAndExit(nRet, "Error: DaSetBoardConfig()\n");
		// アナログ出力設定情報読み出し（次の設定で全パラメータを代入するので、読み出しは省略）
//	nRet = DaGetSamplingConfig(DA1_NUM, pDaSmplConfig);
//	if(nRet != DA_ERROR_SUCCESS) CloseAndExit(nRet, "Error: DaGetSamplingConfig()\n");
						// アナログ出力設定情報設定
						DaSmplConfig.ulChCount = 2;
						DaSmplConfig.SmplChReq[0].ulChNo = 1;
						DaSmplConfig.SmplChReq[0].ulRange = DA_0_5V;
						DaSmplConfig.SmplChReq[1].ulChNo = 2;
						DaSmplConfig.SmplChReq[1].ulRange = DA_0_5V;
						//	DaSmplConfig.SmplChReq[2].ulChNo = 3;
//	DaSmplConfig.SmplChReq[2].ulRange = DA_5V;
						//	DaSmplConfig.SmplChReq[3].ulChNo = 4;
//	DaSmplConfig.SmplChReq[3].ulRange = DA_5V;
						DaSmplConfig.ulSamplingMode = DA_MEM_SAMPLING;
						DaSmplConfig.fSmplFreq = DA_SMPL_FREQ;
						DaSmplConfig.ulSmplRepeat = 1;
						DaSmplConfig.ulTrigMode = DA_FREERUN;
						DaSmplConfig.ulTrigPoint = DA_TRIG_START;
						DaSmplConfig.ulTrigDelay = 0;
						DaSmplConfig.ulEClkEdge = DA_DOWN_EDGE;
						DaSmplConfig.ulTrigEdge = DA_DOWN_EDGE;
						DaSmplConfig.ulTrigDI = 0;
						nRet = DaSetSamplingConfig(1, &DaSmplConfig);
						if(nRet != DA_ERROR_SUCCESS) CloseAndExit(nRet, "Error: DaSetSamplingConfig()\n");
}
void main(void){

  AdInit();
	AdSetup();
	DaInit();

  AdStart();
//	DaStart(FLAG_SYNC);	// DAが出力完了するまで帰ってこない


//	nRet = DaOutputDO(DA1_NUM, 0x01);

//	unsigned short init_data[4] = {0x1000,0x2000,0x3000,0x4000};
//	nRet = DaOutputDA(DA1_NUM, 2, DaSmplConfig.SmplChReq, init_data);

//	fputs("#1\n",stdout);
//	fgetc(stdin);

//	DA変換データを作成



	unsigned short *data = (unsigned short *)DaSmplData;
	unsigned long lx = 0;
	unsigned long V2d = 0xFF80;
	unsigned long V2d_half = V2d / 2;
	nDaCount = 0;

//	fprintf(stderr, "nDaCount=%d\n", nDaCount);
	for(i=0; i<256; i+=1){
		*data++ = 0 ;  // ch1
		if(20<i && i<40){  // ch2
			*data++ = 0xFC00;
		} else {
			*data++ = 0;
		}
		nDaCount++;
	//	fprintf(stderr, "%04x, %04lx\n", nDaCount, V2d_half);
	}
	fprintf(stderr, "[1] %d\n", nDaCount);

	for(j=0; j<1792; j+=1){
		*data++ = lx ;  // ch1
		lx += 0x24 ;
		*dada++ = 0;  // ch2
		nDaCount++;
	}
	fprintf(stderr, "[2] %d\n", nDaCount);

	for(k=0; k<1792; k+=1){
		*data++ = lx ;  // ch1
		lx -= 0x24 ;
		*dada++ = 0;  // ch2
		nDaCount++;
	}
	fprintf(stderr, "[3] %d\n", nDaCount);

	for(l=0; l<256; l+=1){
		*data++ = 0;  // ch1
		*data++ = 0;  // ch2
		nDaCount++;
	}
	fprintf(stderr, "[4] %d\n", nDaCount);

//	for(lX=0; lX<0x20000; lX+=0x100){
	//	*data++ = 0;
	//	*data++ = 0;
		// ch1
	//	if(lX<0x10000){
		//	*data++ = lX;



	//	}else{
	//		*data++ = 0x1FFFF - lX;
	//	}


	//	printf("%x\n",lX);



		// ch2
	 //	if(lX < 0x20){
		//	*data++ = 0xFFFF;
		//} else {
		//	*data++ = 0x0000;
		//}
	//}

	nRet = DaClearSamplingData(DA1_NUM);
	if(nRet != DA_ERROR_SUCCESS) CloseAndExit(nRet, "Error: DaClearSamplingData()\n");
	nRet = DaSetSamplingData(DA1_NUM, DaSmplData, nDaCount);
	if(nRet != DA_ERROR_SUCCESS) CloseAndExit(nRet, "Error: DaSetSamplingData()\n");

	fputs("#2\n",stdout);
	fgetc(stdin);

	nRet = DaStartSampling(DA1_NUM, FLAG_ASYNC);
	if(nRet != DA_ERROR_SUCCESS) CloseAndExit(nRet, "Error: DaStartSampling()\n");

	fputs("#3\n",stdout);
	fgetc(stdin);

//	nRet = DaOutputDO(DA1_NUM, 0x00);

//	fputs("#4\n",stdout);
//	fgetc(stdin);

  usleep(2000);	// ADはまだ走っているので2ms待つ　なくてもよい

  AdGetData();

	CloseAndExit(0, "測定終了\n");
}
