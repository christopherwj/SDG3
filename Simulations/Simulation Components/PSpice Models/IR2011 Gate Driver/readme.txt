##########################################################################################
## Simulation model of IR2011 Level 1 for SIMetrix version 8.3g or higher
##########################################################################################

--------------------------------------------------------------------------------
1. Changelog
--------------------------------------------------------------------------------
VER    | BY                       | DATE          | OBS                         
--------------------------------------------------------------------------------
01.00  | MA                       | 2020-09-11    | created    (*1)
--------------------------------------------------------------------------------

(*1) 
Simulation model of IR2011 Level 1 for SIMetrix version 8.3g or higher
Template performance : 
  - Static Electrical Characteristics and Dynamic Electrical Characteristics 
    are modeled with the typical values from the datasheet.
  - Temperature effects are not modeled
The following features have been modeled :
  - Switching Characteristics such as propagation delay, peak/mean currents
  - Undervoltage lockout 

--------------------------------------------------------------------------------
2. Supported platform
--------------------------------------------------------------------------------
This model has been successfully tested with the following software:
- SIMetrix 8.3g or newer versions

--------------------------------------------------------------------------------
3. Structure of the delivery package
--------------------------------------------------------------------------------
The delivery package contains the following:

	\Documentation\GettingStarted.pdf         : guidance on how to get started with the attached behavioral model
	\Documentation\Disclaimer.pdf             : Infineon’s model terms of use
	\Model\IR2011.lib                         : model lib file
	\Model\IR2011.sxslb                       : schematic symbol for the lib
	\Model\DEMO_TESTBENCH.sxsch               : demo testbench provided together with the model 
	\Model\IGBT_650V_TRENCHSTOP5_F5_L1.lib    : Infineon's IGBT model
	\Model\IGBT_650V_TRENCHSTOP5_F5_L1.sxslb  : Infineon's IGBT symbol for schematic
	\Model\results                            : some screenshots with the demo testbench and results
	
--------------------------------------------------------------------------------
4. Infineon contact and support
--------------------------------------------------------------------------------
support@infineon.com

--------------------------------------------------------------------------------
5. Disclaimer
--------------------------------------------------------------------------------

INFINEON’S MODEL TERMS OF USE

BY DOWNLOADING AND/OR USING THIS INFINEON MODEL (“MODEL”), THE USER (INCLUDING YOU) 
AGREES TO BE BOUND BY THE TERMS OF USE HERE STATED. IF USER DOES NOT AGREE TO ALL TERMS OF
USE HERE STATED, USER SHALL NOT DOWNLOAD, USE OR COPY THE MODEL BUT IMMEDIATELY DELETE 
IT (TO THE EXTENT THAT IT WAS DOWNLOADED ALREADY).

1. SCOPE OF USE
1.1 Any use of this Model provided by Infineon Technologies AG is subject to these Terms of Use.
1.2 This Model, provided by Infineon, does not fully represent all of the specifications and
operating characteristics of the product to which this Model relates.
1.3 This Model only describes the characteristics of a typical product. In all cases, the current data
sheet information for a given product is the final design guideline and the only actual 
performance specification. Although this Model can be a useful tool to evaluate the product 
performance, it cannot simulate the exact product performance under all conditions and it is 
also not intended to replace bread-boarding for final verification.

2. IMPORTANT NOTICE
2.1 Infineon Technologies AG (“Infineon”) is not and cannot be aware of the specific application of 
the Infineon’s Model by User. However, Model may from time to time be used by User in potentially 
harmful and/or life-endangering applications such as traffic, logistic, medical, nuclear or military a
pplications or in other applications where failure of the Model may predictably cause damage to 
persons’ life or health or to property (hereinafter "Critical Applications").
2.2 User acknowledges that Infineon has not specifically designed or qualified the Model for 
Critical Applications that the Model may contain errors and bugs and that User is required to 
qualify the Model for Critical Applications pursuant to the applicable local quality, safety and 
legal requirements before permitting or giving access to any such use.

3. CONFIDENTIAL INFORMATION
User shall treat ideas, concepts and information incorporated in the Model, the documentation and 
the content of this Terms of Use (together hereinafter "Confidential Information") confidential, not 
disclose it to any third party unless otherwise agreed in writing between User and Infineon, not use 
it for any other purposes than using the Model for simulation and testing purposes only.

4. WARRANTY
4.1 User acknowledges that the Model is provided by Infineon under this Terms of Use is provided free 
of charge and "AS IS" without any warranty or liability of any kind and Infineon hereby expressly 
disclaims any warranties or representations, whether express, implied, statutory or otherwise, 
including but not limited to warranties of workmanship, merchantability, fitness for a particular 
purpose, defects in the Model, or non-infringement of third parties intellectual property rights.
4.2 Infineon reserves the right to make corrections, deletions, modifications, enhancements, 
improvements and other changes to the Model at any time or to move or discontinue any Model without notice.

5. LIABILITY
5.1 Nothing in this Terms of Use shall limit or exclude Infineon's liability under mandatory liability 
laws, for injuries to life, body or health, for fraudulent concealment of defects in the software, or 
in cases of Infineon's intentional misconduct or gross negligence.
5.2 Without prejudice to Sections 5.1, in cases of Infineon's slight negligent breach of obligations 
that restrict essential rights or duties arising from the nature of this Terms of Use in a way that 
there is a risk of non-achievement of the purpose of this Terms of Use or of an obligation whose observance
User regularly may trust in and whereas compliance with only makes proper execution of this Terms 
of Use possible, Infineon's liability shall be limited to the typically, foreseeable damage.
5.3 Without prejudice to Sections 8.1 and 8.2, Infineon's liability under this Agreement shall be excluded in all 
other cases.

6. EXPORT REGULATIONS
The User shall comply with all applicable national and international laws and regulations, in 
particular the applicable export control regulations and sanction programs. The User also agrees 
not to export, re-export or transfer any software or technology developed with or using information, 
software or technology offered by Infineon, in violation of any applicable laws or regulations of 
the competent authorities. Further, the User shall neither use any products, information, software 
and technology offered by Infineon in or in connection with nuclear technology or weapons of mass 
destruction (nuclear, biological or chemical) and carriers thereof nor supply military consignees.

7. TERMINATION OF USE PERMIT
If the User violates these Terms of Use, such User’s permit to use this Model terminates automatically. 
In addition, Infineon may terminate the User’s permit to use this Model at its discretion and at 
any time regardless of any violation of these Terms of Use. In any of the foregoing events, the User 
is obliged to immediately destroy any content that has been downloaded or printed from Infineon’s website.

8. MISCELLANEOUS
8.1 These Terms of Use are subject to the laws of the Federal Republic of Germany with the exception 
of the United Nations on Purchase Contracts on the International Sale of Goods dated April 11, 1980 (CISG). 
The exclusive place of jurisdiction is Munich, Germany.
8.2 Should any provision in these Terms of Use be or become invalid, the validity of all other 
provisions or agreements shall remain unaffected thereby.
