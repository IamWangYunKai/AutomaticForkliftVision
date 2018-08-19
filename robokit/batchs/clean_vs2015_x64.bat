@rem ************************************************
@rem * Script to clean the solutions of robokit    *
@rem * Created by: Ye Yangsheng                    *
@rem * Created 2017.03.04                          *
@rem * © 2015-2017 Seer Robotics Co,.Ltd.          *
@rem * http://www.seer-robotics.com                *
@rem ************************************************

@echo ################################################################################
@echo #######################  RoboKit MSVC Clean Batch ##############################
@echo ###############  Copyright 2015-2017 Seer Robotics Co,.Ltd.  ###################
@echo ################################################################################
@echo clean start ...
@call vcvars64_2015.bat
@if not exist ../build echo build directory not exist & goto end
@cd ../build
@set _log="cleanResults.log"
@echo [%DATE% %Time%] Start clean sequence >%_log%
@echo Used clean configuration is %buildAnyCPU% >>%_log%
@rem Start clean************************************************
@set _solution_file="RoboKit.sln"
@devenv.com %_solution_file% /clean "Release|x64" /Out %_log%
@if not %errorlevel% == 0 echo %_solution_file% failed!   Error: %errorlevel% >>%_log%
@if %errorlevel% == 0 echo %_solution_file% clean successful >>%_log%
@echo [%DATE% %Time%] Finished clean sequence >>%_log%
@rem sleep 3 seconds
@timeout /t 3
@cd ../batchs
:end
