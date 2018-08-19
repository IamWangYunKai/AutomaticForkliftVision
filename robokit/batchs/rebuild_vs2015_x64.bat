@rem ************************************************
@rem * Script to recompile the solutions of robokit  *
@rem * Created by: Ye Yangsheng                    *
@rem * Created 2017.03.04                          *
@rem * © 2015-2017 Seer Robotics Co,.Ltd.          *
@rem * http://www.seer-robotics.com                *
@rem ************************************************

@echo ################################################################################
@echo #######################  RoboKit MSVC Build Batch ##############################
@echo ###############  Copyright 2015-2017 Seer Robotics Co,.Ltd.  ###################
@echo ################################################################################
@echo rebuild start ...
@call vcvars64_2015.bat
@if not exist ../build echo build directory not exist & goto end
@cd ../build
@set _log="recompileResults.log"
@echo [%DATE% %Time%] Start recompile sequence >%_log%
@echo Used recompile configuration is %buildAnyCPU% >>%_log%
@rem Start recompile************************************************
@set _solution_file="RoboKit.sln"
@devenv.com %_solution_file% /rebuild "Release|x64" /Out %_log%
@if not %errorlevel% == 0 echo %_solution_file% failed!   Error: %errorlevel% >>%_log%
@if %errorlevel% == 0 echo %_solution_file% recompiled successful >>%_log%
@echo [%DATE% %Time%] Finished recompile sequence >>%_log%
@rem sleep 3 seconds
@timeout /t 3
@cd ../batchs
:end
