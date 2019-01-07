/*
 * cli.h
 *
 *      Author: SHunt
 */

#ifndef CLI_H_
#define CLI_H_


/******************************  Variables  **********************************/
extern uint8_t		cmdReceived;

/****************************** Internal types ********************************/

typedef  void (*cmdFunc)(uint8_t *);

/*************************** Functions prototypes *****************************/
void DoNothing();
void CmdHelp(uint8_t *args);
uint8_t *FindArgv(uint8_t *args);
void GetArgv(char *dst, uint8_t *args);
void CmdProcess(void);
void CmdPrompt(void);
cmdFunc FindCommand(char *cmd);
void SetDefaultSns(uint8_t *args);
void SnsConnected(uint8_t *args);
void RdCfgs(uint8_t *args);
void RdTemp(uint8_t *args);
void RdHum(uint8_t *args);
void CfgMeasTime(uint8_t *args);
void StartMeas(uint8_t *args);
void StopMeas(uint8_t *args);
void CfgRtia(uint8_t *args);
void CfgRload(uint8_t *args);
void CfgVbias(uint8_t *args);
void CfgSens(uint8_t *args);
void CfgTempComp(uint8_t *args);
void RunEIS(uint8_t *args);
void RdEIS(uint8_t *args);
void RdEISfull(uint8_t *args);
void RdRcal(uint8_t *args);
void RunPulse(uint8_t *args);
void ReadPulse(uint8_t *args);
void SetPulseAmpl(uint8_t *args);
void SetPulseDuration(uint8_t *args);
void RdSensors(uint8_t *args);
void StopRd(uint8_t *args);
void CfgUpdateRate(uint8_t *args);






#endif /* CLI_H_ */
