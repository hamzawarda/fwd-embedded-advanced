#ifndef __TM4C123GH6PM_H__
#define __TM4C123GH6PM_H__

/*****************************************************************************

 GPIO registers (PORTF)

*****************************************************************************/
#define GPIO_PORTF_DATA_BITS_R  ((volatile uint32_t *)0x40025000)
#define GPIO_PORTF_DATA_R       (*((volatile uint32_t *)0x400253FC))
#define GPIO_PORTF_DIR_R        (*((volatile uint32_t *)0x40025400))
#define GPIO_PORTF_IS_R         (*((volatile uint32_t *)0x40025404))
#define GPIO_PORTF_IBE_R        (*((volatile uint32_t *)0x40025408))
#define GPIO_PORTF_IEV_R        (*((volatile uint32_t *)0x4002540C))
#define GPIO_PORTF_IM_R         (*((volatile uint32_t *)0x40025410))
#define GPIO_PORTF_RIS_R        (*((volatile uint32_t *)0x40025414))
#define GPIO_PORTF_MIS_R        (*((volatile uint32_t *)0x40025418))
#define GPIO_PORTF_ICR_R        (*((volatile uint32_t *)0x4002541C))
#define GPIO_PORTF_AFSEL_R      (*((volatile uint32_t *)0x40025420))
#define GPIO_PORTF_DR2R_R       (*((volatile uint32_t *)0x40025500))
#define GPIO_PORTF_DR4R_R       (*((volatile uint32_t *)0x40025504))
#define GPIO_PORTF_DR8R_R       (*((volatile uint32_t *)0x40025508))
#define GPIO_PORTF_ODR_R        (*((volatile uint32_t *)0x4002550C))
#define GPIO_PORTF_PUR_R        (*((volatile uint32_t *)0x40025510))
#define GPIO_PORTF_PDR_R        (*((volatile uint32_t *)0x40025514))
#define GPIO_PORTF_SLR_R        (*((volatile uint32_t *)0x40025518))
#define GPIO_PORTF_DEN_R        (*((volatile uint32_t *)0x4002551C))
#define GPIO_PORTF_LOCK_R       (*((volatile uint32_t *)0x40025520))
#define GPIO_PORTF_CR_R         (*((volatile uint32_t *)0x40025524))
#define GPIO_PORTF_AMSEL_R      (*((volatile uint32_t *)0x40025528))
#define GPIO_PORTF_PCTL_R       (*((volatile uint32_t *)0x4002552C))
#define GPIO_PORTF_ADCCTL_R     (*((volatile uint32_t *)0x40025530))
#define GPIO_PORTF_DMACTL_R     (*((volatile uint32_t *)0x40025534))

/*****************************************************************************

 System Control registers (SYSCTL)

*****************************************************************************/
#define SYSCTL_DID0_R           (*((volatile uint32_t *)0x400FE000))
#define SYSCTL_DID1_R           (*((volatile uint32_t *)0x400FE004))
#define SYSCTL_DC0_R            (*((volatile uint32_t *)0x400FE008))
#define SYSCTL_DC1_R            (*((volatile uint32_t *)0x400FE010))
#define SYSCTL_DC2_R            (*((volatile uint32_t *)0x400FE014))
#define SYSCTL_DC3_R            (*((volatile uint32_t *)0x400FE018))
#define SYSCTL_DC4_R            (*((volatile uint32_t *)0x400FE01C))
#define SYSCTL_DC5_R            (*((volatile uint32_t *)0x400FE020))
#define SYSCTL_DC6_R            (*((volatile uint32_t *)0x400FE024))
#define SYSCTL_DC7_R            (*((volatile uint32_t *)0x400FE028))
#define SYSCTL_DC8_R            (*((volatile uint32_t *)0x400FE02C))
#define SYSCTL_PBORCTL_R        (*((volatile uint32_t *)0x400FE030))
#define SYSCTL_SRCR0_R          (*((volatile uint32_t *)0x400FE040))
#define SYSCTL_SRCR1_R          (*((volatile uint32_t *)0x400FE044))
#define SYSCTL_SRCR2_R          (*((volatile uint32_t *)0x400FE048))
#define SYSCTL_RIS_R            (*((volatile uint32_t *)0x400FE050))
#define SYSCTL_IMC_R            (*((volatile uint32_t *)0x400FE054))
#define SYSCTL_MISC_R           (*((volatile uint32_t *)0x400FE058))
#define SYSCTL_RESC_R           (*((volatile uint32_t *)0x400FE05C))
#define SYSCTL_RCC_R            (*((volatile uint32_t *)0x400FE060))
#define SYSCTL_GPIOHBCTL_R      (*((volatile uint32_t *)0x400FE06C))
#define SYSCTL_RCC2_R           (*((volatile uint32_t *)0x400FE070))
#define SYSCTL_MOSCCTL_R        (*((volatile uint32_t *)0x400FE07C))
#define SYSCTL_RCGC0_R          (*((volatile uint32_t *)0x400FE100))
#define SYSCTL_RCGC1_R          (*((volatile uint32_t *)0x400FE104))
#define SYSCTL_RCGC2_R          (*((volatile uint32_t *)0x400FE108))
#define SYSCTL_SCGC0_R          (*((volatile uint32_t *)0x400FE110))
#define SYSCTL_SCGC1_R          (*((volatile uint32_t *)0x400FE114))
#define SYSCTL_SCGC2_R          (*((volatile uint32_t *)0x400FE118))
#define SYSCTL_DCGC0_R          (*((volatile uint32_t *)0x400FE120))
#define SYSCTL_DCGC1_R          (*((volatile uint32_t *)0x400FE124))
#define SYSCTL_DCGC2_R          (*((volatile uint32_t *)0x400FE128))
#define SYSCTL_DSLPCLKCFG_R     (*((volatile uint32_t *)0x400FE144))
#define SYSCTL_SYSPROP_R        (*((volatile uint32_t *)0x400FE14C))
#define SYSCTL_PIOSCCAL_R       (*((volatile uint32_t *)0x400FE150))
#define SYSCTL_PIOSCSTAT_R      (*((volatile uint32_t *)0x400FE154))
#define SYSCTL_PLLFREQ0_R       (*((volatile uint32_t *)0x400FE160))
#define SYSCTL_PLLFREQ1_R       (*((volatile uint32_t *)0x400FE164))
#define SYSCTL_PLLSTAT_R        (*((volatile uint32_t *)0x400FE168))
#define SYSCTL_SLPPWRCFG_R      (*((volatile uint32_t *)0x400FE188))
#define SYSCTL_DSLPPWRCFG_R     (*((volatile uint32_t *)0x400FE18C))
#define SYSCTL_DC9_R            (*((volatile uint32_t *)0x400FE190))
#define SYSCTL_NVMSTAT_R        (*((volatile uint32_t *)0x400FE1A0))
#define SYSCTL_LDOSPCTL_R       (*((volatile uint32_t *)0x400FE1B4))
#define SYSCTL_LDODPCTL_R       (*((volatile uint32_t *)0x400FE1BC))
#define SYSCTL_PPWD_R           (*((volatile uint32_t *)0x400FE300))
#define SYSCTL_PPTIMER_R        (*((volatile uint32_t *)0x400FE304))
#define SYSCTL_PPGPIO_R         (*((volatile uint32_t *)0x400FE308))
#define SYSCTL_PPDMA_R          (*((volatile uint32_t *)0x400FE30C))
#define SYSCTL_PPHIB_R          (*((volatile uint32_t *)0x400FE314))
#define SYSCTL_PPUART_R         (*((volatile uint32_t *)0x400FE318))
#define SYSCTL_PPSSI_R          (*((volatile uint32_t *)0x400FE31C))
#define SYSCTL_PPI2C_R          (*((volatile uint32_t *)0x400FE320))
#define SYSCTL_PPUSB_R          (*((volatile uint32_t *)0x400FE328))
#define SYSCTL_PPCAN_R          (*((volatile uint32_t *)0x400FE334))
#define SYSCTL_PPADC_R          (*((volatile uint32_t *)0x400FE338))
#define SYSCTL_PPACMP_R         (*((volatile uint32_t *)0x400FE33C))
#define SYSCTL_PPPWM_R          (*((volatile uint32_t *)0x400FE340))
#define SYSCTL_PPQEI_R          (*((volatile uint32_t *)0x400FE344))
#define SYSCTL_PPEEPROM_R       (*((volatile uint32_t *)0x400FE358))
#define SYSCTL_PPWTIMER_R       (*((volatile uint32_t *)0x400FE35C))
#define SYSCTL_SRWD_R           (*((volatile uint32_t *)0x400FE500))
#define SYSCTL_SRTIMER_R        (*((volatile uint32_t *)0x400FE504))
#define SYSCTL_SRGPIO_R         (*((volatile uint32_t *)0x400FE508))
#define SYSCTL_SRDMA_R          (*((volatile uint32_t *)0x400FE50C))
#define SYSCTL_SRHIB_R          (*((volatile uint32_t *)0x400FE514))
#define SYSCTL_SRUART_R         (*((volatile uint32_t *)0x400FE518))
#define SYSCTL_SRSSI_R          (*((volatile uint32_t *)0x400FE51C))
#define SYSCTL_SRI2C_R          (*((volatile uint32_t *)0x400FE520))
#define SYSCTL_SRUSB_R          (*((volatile uint32_t *)0x400FE528))
#define SYSCTL_SRCAN_R          (*((volatile uint32_t *)0x400FE534))
#define SYSCTL_SRADC_R          (*((volatile uint32_t *)0x400FE538))
#define SYSCTL_SRACMP_R         (*((volatile uint32_t *)0x400FE53C))
#define SYSCTL_SRPWM_R          (*((volatile uint32_t *)0x400FE540))
#define SYSCTL_SRQEI_R          (*((volatile uint32_t *)0x400FE544))
#define SYSCTL_SREEPROM_R       (*((volatile uint32_t *)0x400FE558))
#define SYSCTL_SRWTIMER_R       (*((volatile uint32_t *)0x400FE55C))
#define SYSCTL_RCGCWD_R         (*((volatile uint32_t *)0x400FE600))
#define SYSCTL_RCGCTIMER_R      (*((volatile uint32_t *)0x400FE604))
#define SYSCTL_RCGCGPIO_R       (*((volatile uint32_t *)0x400FE608))
#define SYSCTL_RCGCDMA_R        (*((volatile uint32_t *)0x400FE60C))
#define SYSCTL_RCGCHIB_R        (*((volatile uint32_t *)0x400FE614))
#define SYSCTL_RCGCUART_R       (*((volatile uint32_t *)0x400FE618))
#define SYSCTL_RCGCSSI_R        (*((volatile uint32_t *)0x400FE61C))
#define SYSCTL_RCGCI2C_R        (*((volatile uint32_t *)0x400FE620))
#define SYSCTL_RCGCUSB_R        (*((volatile uint32_t *)0x400FE628))
#define SYSCTL_RCGCCAN_R        (*((volatile uint32_t *)0x400FE634))
#define SYSCTL_RCGCADC_R        (*((volatile uint32_t *)0x400FE638))
#define SYSCTL_RCGCACMP_R       (*((volatile uint32_t *)0x400FE63C))
#define SYSCTL_RCGCPWM_R        (*((volatile uint32_t *)0x400FE640))
#define SYSCTL_RCGCQEI_R        (*((volatile uint32_t *)0x400FE644))
#define SYSCTL_RCGCEEPROM_R     (*((volatile uint32_t *)0x400FE658))
#define SYSCTL_RCGCWTIMER_R     (*((volatile uint32_t *)0x400FE65C))
#define SYSCTL_SCGCWD_R         (*((volatile uint32_t *)0x400FE700))
#define SYSCTL_SCGCTIMER_R      (*((volatile uint32_t *)0x400FE704))
#define SYSCTL_SCGCGPIO_R       (*((volatile uint32_t *)0x400FE708))
#define SYSCTL_SCGCDMA_R        (*((volatile uint32_t *)0x400FE70C))
#define SYSCTL_SCGCHIB_R        (*((volatile uint32_t *)0x400FE714))
#define SYSCTL_SCGCUART_R       (*((volatile uint32_t *)0x400FE718))
#define SYSCTL_SCGCSSI_R        (*((volatile uint32_t *)0x400FE71C))
#define SYSCTL_SCGCI2C_R        (*((volatile uint32_t *)0x400FE720))
#define SYSCTL_SCGCUSB_R        (*((volatile uint32_t *)0x400FE728))
#define SYSCTL_SCGCCAN_R        (*((volatile uint32_t *)0x400FE734))
#define SYSCTL_SCGCADC_R        (*((volatile uint32_t *)0x400FE738))
#define SYSCTL_SCGCACMP_R       (*((volatile uint32_t *)0x400FE73C))
#define SYSCTL_SCGCPWM_R        (*((volatile uint32_t *)0x400FE740))
#define SYSCTL_SCGCQEI_R        (*((volatile uint32_t *)0x400FE744))
#define SYSCTL_SCGCEEPROM_R     (*((volatile uint32_t *)0x400FE758))
#define SYSCTL_SCGCWTIMER_R     (*((volatile uint32_t *)0x400FE75C))
#define SYSCTL_DCGCWD_R         (*((volatile uint32_t *)0x400FE800))
#define SYSCTL_DCGCTIMER_R      (*((volatile uint32_t *)0x400FE804))
#define SYSCTL_DCGCGPIO_R       (*((volatile uint32_t *)0x400FE808))
#define SYSCTL_DCGCDMA_R        (*((volatile uint32_t *)0x400FE80C))
#define SYSCTL_DCGCHIB_R        (*((volatile uint32_t *)0x400FE814))
#define SYSCTL_DCGCUART_R       (*((volatile uint32_t *)0x400FE818))
#define SYSCTL_DCGCSSI_R        (*((volatile uint32_t *)0x400FE81C))
#define SYSCTL_DCGCI2C_R        (*((volatile uint32_t *)0x400FE820))
#define SYSCTL_DCGCUSB_R        (*((volatile uint32_t *)0x400FE828))
#define SYSCTL_DCGCCAN_R        (*((volatile uint32_t *)0x400FE834))
#define SYSCTL_DCGCADC_R        (*((volatile uint32_t *)0x400FE838))
#define SYSCTL_DCGCACMP_R       (*((volatile uint32_t *)0x400FE83C))
#define SYSCTL_DCGCPWM_R        (*((volatile uint32_t *)0x400FE840))
#define SYSCTL_DCGCQEI_R        (*((volatile uint32_t *)0x400FE844))
#define SYSCTL_DCGCEEPROM_R     (*((volatile uint32_t *)0x400FE858))
#define SYSCTL_DCGCWTIMER_R     (*((volatile uint32_t *)0x400FE85C))
#define SYSCTL_PRWD_R           (*((volatile uint32_t *)0x400FEA00))
#define SYSCTL_PRTIMER_R        (*((volatile uint32_t *)0x400FEA04))
#define SYSCTL_PRGPIO_R         (*((volatile uint32_t *)0x400FEA08))
#define SYSCTL_PRDMA_R          (*((volatile uint32_t *)0x400FEA0C))
#define SYSCTL_PRHIB_R          (*((volatile uint32_t *)0x400FEA14))
#define SYSCTL_PRUART_R         (*((volatile uint32_t *)0x400FEA18))
#define SYSCTL_PRSSI_R          (*((volatile uint32_t *)0x400FEA1C))
#define SYSCTL_PRI2C_R          (*((volatile uint32_t *)0x400FEA20))
#define SYSCTL_PRUSB_R          (*((volatile uint32_t *)0x400FEA28))
#define SYSCTL_PRCAN_R          (*((volatile uint32_t *)0x400FEA34))
#define SYSCTL_PRADC_R          (*((volatile uint32_t *)0x400FEA38))
#define SYSCTL_PRACMP_R         (*((volatile uint32_t *)0x400FEA3C))
#define SYSCTL_PRPWM_R          (*((volatile uint32_t *)0x400FEA40))
#define SYSCTL_PRQEI_R          (*((volatile uint32_t *)0x400FEA44))
#define SYSCTL_PREEPROM_R       (*((volatile uint32_t *)0x400FEA58))
#define SYSCTL_PRWTIMER_R       (*((volatile uint32_t *)0x400FEA5C))

/*****************************************************************************

 NVIC registers (NVIC)

*****************************************************************************/
#define NVIC_ACTLR_R            (*((volatile uint32_t *)0xE000E008))
#define NVIC_ST_CTRL_R          (*((volatile uint32_t *)0xE000E010))
#define NVIC_ST_RELOAD_R        (*((volatile uint32_t *)0xE000E014))
#define NVIC_ST_CURRENT_R       (*((volatile uint32_t *)0xE000E018))
#define NVIC_EN0_R              (*((volatile uint32_t *)0xE000E100))
#define NVIC_EN1_R              (*((volatile uint32_t *)0xE000E104))
#define NVIC_EN2_R              (*((volatile uint32_t *)0xE000E108))
#define NVIC_EN3_R              (*((volatile uint32_t *)0xE000E10C))
#define NVIC_EN4_R              (*((volatile uint32_t *)0xE000E110))
#define NVIC_DIS0_R             (*((volatile uint32_t *)0xE000E180))
#define NVIC_DIS1_R             (*((volatile uint32_t *)0xE000E184))
#define NVIC_DIS2_R             (*((volatile uint32_t *)0xE000E188))
#define NVIC_DIS3_R             (*((volatile uint32_t *)0xE000E18C))
#define NVIC_DIS4_R             (*((volatile uint32_t *)0xE000E190))
#define NVIC_PEND0_R            (*((volatile uint32_t *)0xE000E200))
#define NVIC_PEND1_R            (*((volatile uint32_t *)0xE000E204))
#define NVIC_PEND2_R            (*((volatile uint32_t *)0xE000E208))
#define NVIC_PEND3_R            (*((volatile uint32_t *)0xE000E20C))
#define NVIC_PEND4_R            (*((volatile uint32_t *)0xE000E210))
#define NVIC_UNPEND0_R          (*((volatile uint32_t *)0xE000E280))
#define NVIC_UNPEND1_R          (*((volatile uint32_t *)0xE000E284))
#define NVIC_UNPEND2_R          (*((volatile uint32_t *)0xE000E288))
#define NVIC_UNPEND3_R          (*((volatile uint32_t *)0xE000E28C))
#define NVIC_UNPEND4_R          (*((volatile uint32_t *)0xE000E290))
#define NVIC_ACTIVE0_R          (*((volatile uint32_t *)0xE000E300))
#define NVIC_ACTIVE1_R          (*((volatile uint32_t *)0xE000E304))
#define NVIC_ACTIVE2_R          (*((volatile uint32_t *)0xE000E308))
#define NVIC_ACTIVE3_R          (*((volatile uint32_t *)0xE000E30C))
#define NVIC_ACTIVE4_R          (*((volatile uint32_t *)0xE000E310))
#define NVIC_PRI0_R             (*((volatile uint32_t *)0xE000E400))
#define NVIC_PRI1_R             (*((volatile uint32_t *)0xE000E404))
#define NVIC_PRI2_R             (*((volatile uint32_t *)0xE000E408))
#define NVIC_PRI3_R             (*((volatile uint32_t *)0xE000E40C))
#define NVIC_PRI4_R             (*((volatile uint32_t *)0xE000E410))
#define NVIC_PRI5_R             (*((volatile uint32_t *)0xE000E414))
#define NVIC_PRI6_R             (*((volatile uint32_t *)0xE000E418))
#define NVIC_PRI7_R             (*((volatile uint32_t *)0xE000E41C))
#define NVIC_PRI8_R             (*((volatile uint32_t *)0xE000E420))
#define NVIC_PRI9_R             (*((volatile uint32_t *)0xE000E424))
#define NVIC_PRI10_R            (*((volatile uint32_t *)0xE000E428))
#define NVIC_PRI11_R            (*((volatile uint32_t *)0xE000E42C))
#define NVIC_PRI12_R            (*((volatile uint32_t *)0xE000E430))
#define NVIC_PRI13_R            (*((volatile uint32_t *)0xE000E434))
#define NVIC_PRI14_R            (*((volatile uint32_t *)0xE000E438))
#define NVIC_PRI15_R            (*((volatile uint32_t *)0xE000E43C))
#define NVIC_PRI16_R            (*((volatile uint32_t *)0xE000E440))
#define NVIC_PRI17_R            (*((volatile uint32_t *)0xE000E444))
#define NVIC_PRI18_R            (*((volatile uint32_t *)0xE000E448))
#define NVIC_PRI19_R            (*((volatile uint32_t *)0xE000E44C))
#define NVIC_PRI20_R            (*((volatile uint32_t *)0xE000E450))
#define NVIC_PRI21_R            (*((volatile uint32_t *)0xE000E454))
#define NVIC_PRI22_R            (*((volatile uint32_t *)0xE000E458))
#define NVIC_PRI23_R            (*((volatile uint32_t *)0xE000E45C))
#define NVIC_PRI24_R            (*((volatile uint32_t *)0xE000E460))
#define NVIC_PRI25_R            (*((volatile uint32_t *)0xE000E464))
#define NVIC_PRI26_R            (*((volatile uint32_t *)0xE000E468))
#define NVIC_PRI27_R            (*((volatile uint32_t *)0xE000E46C))
#define NVIC_PRI28_R            (*((volatile uint32_t *)0xE000E470))
#define NVIC_PRI29_R            (*((volatile uint32_t *)0xE000E474))
#define NVIC_PRI30_R            (*((volatile uint32_t *)0xE000E478))
#define NVIC_PRI31_R            (*((volatile uint32_t *)0xE000E47C))
#define NVIC_PRI32_R            (*((volatile uint32_t *)0xE000E480))
#define NVIC_PRI33_R            (*((volatile uint32_t *)0xE000E484))
#define NVIC_PRI34_R            (*((volatile uint32_t *)0xE000E488))
#define NVIC_CPUID_R            (*((volatile uint32_t *)0xE000ED00))
#define NVIC_INT_CTRL_R         (*((volatile uint32_t *)0xE000ED04))
#define NVIC_VTABLE_R           (*((volatile uint32_t *)0xE000ED08))
#define NVIC_APINT_R            (*((volatile uint32_t *)0xE000ED0C))
#define NVIC_SYS_CTRL_R         (*((volatile uint32_t *)0xE000ED10))
#define NVIC_CFG_CTRL_R         (*((volatile uint32_t *)0xE000ED14))
#define NVIC_SYS_PRI1_R         (*((volatile uint32_t *)0xE000ED18))
#define NVIC_SYS_PRI2_R         (*((volatile uint32_t *)0xE000ED1C))
#define NVIC_SYS_PRI3_R         (*((volatile uint32_t *)0xE000ED20))
#define NVIC_SYS_HND_CTRL_R     (*((volatile uint32_t *)0xE000ED24))
#define NVIC_FAULT_STAT_R       (*((volatile uint32_t *)0xE000ED28))
#define NVIC_HFAULT_STAT_R      (*((volatile uint32_t *)0xE000ED2C))
#define NVIC_DEBUG_STAT_R       (*((volatile uint32_t *)0xE000ED30))
#define NVIC_MM_ADDR_R          (*((volatile uint32_t *)0xE000ED34))
#define NVIC_FAULT_ADDR_R       (*((volatile uint32_t *)0xE000ED38))
#define NVIC_CPAC_R             (*((volatile uint32_t *)0xE000ED88))
#define NVIC_MPU_TYPE_R         (*((volatile uint32_t *)0xE000ED90))
#define NVIC_MPU_CTRL_R         (*((volatile uint32_t *)0xE000ED94))
#define NVIC_MPU_NUMBER_R       (*((volatile uint32_t *)0xE000ED98))
#define NVIC_MPU_BASE_R         (*((volatile uint32_t *)0xE000ED9C))
#define NVIC_MPU_ATTR_R         (*((volatile uint32_t *)0xE000EDA0))
#define NVIC_MPU_BASE1_R        (*((volatile uint32_t *)0xE000EDA4))
#define NVIC_MPU_ATTR1_R        (*((volatile uint32_t *)0xE000EDA8))
#define NVIC_MPU_BASE2_R        (*((volatile uint32_t *)0xE000EDAC))
#define NVIC_MPU_ATTR2_R        (*((volatile uint32_t *)0xE000EDB0))
#define NVIC_MPU_BASE3_R        (*((volatile uint32_t *)0xE000EDB4))
#define NVIC_MPU_ATTR3_R        (*((volatile uint32_t *)0xE000EDB8))
#define NVIC_DBG_CTRL_R         (*((volatile uint32_t *)0xE000EDF0))
#define NVIC_DBG_XFER_R         (*((volatile uint32_t *)0xE000EDF4))
#define NVIC_DBG_DATA_R         (*((volatile uint32_t *)0xE000EDF8))
#define NVIC_DBG_INT_R          (*((volatile uint32_t *)0xE000EDFC))
#define NVIC_SW_TRIG_R          (*((volatile uint32_t *)0xE000EF00))
#define NVIC_FPCC_R             (*((volatile uint32_t *)0xE000EF34))
#define NVIC_FPCA_R             (*((volatile uint32_t *)0xE000EF38))
#define NVIC_FPDSC_R            (*((volatile uint32_t *)0xE000EF3C))

#endif /* __TM4C123GH6PM_H__ */
