; -------------------------------------------------
; 由于中断向量大于 31，在 KEIL中无法直接编译
;而汇编没有问题 
;所以借用第 13 号中断入口地址 来跳转
; -------------------------------------------------
    CSEG AT 0133H  ;P1口中断入口地址
    JMP P1INT_ISR
P1INT_ISR:
    JMP 006BH      ;借用 13 号中断的入口地址
    END


    CSEG AT 0143H ;P3口中断入口地址
    JMP P3INT_ISR
P3INT_ISR:
    JMP 006BH ;借用 13号中断的入口地址
    END