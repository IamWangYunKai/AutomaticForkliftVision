@rem ************************************************
@rem * Script to compile the solutions of robokit  *
@rem * Created by: Ye Yangsheng                    *
@rem * Created 2017.03.04                          *
@rem * © 2015-2017 Seer Robotics Co,.Ltd.          *
@rem * http://www.seer-robotics.com                *
@rem ************************************************

@echo ################################################################################
@echo #######################  RoboKit MSVC Build Batch ##############################
@echo ###############  Copyright 2015-2017 Seer Robotics Co,.Ltd.  ###################
@echo ################################################################################
@echo build start ...
@call vcvars32_2010.bat
@if not exist ../build echo build directory not exist & goto end
@cd ../build
@set _log="compileResults.log"
@echo [%DATE% %Time%] Start compile sequence >%_log%
@echo Used compile configuration is %buildAnyCPU% >>%_log%
@rem Start compile************************************************
@set _solution_file="RoboKit.sln"
@devenv.com %_solution_file% /build "Release|Win32" /Out %_log%
@if not %errorlevel% == 0 echo %_solution_file% failed!   Error: %errorlevel% >>%_log%
@if %errorlevel% == 0 echo %_solution_file% compiled successful >>%_log%
@echo [%DATE% %Time%] Finished compile sequence >>%_log%
@rem sleep 3 seconds
@timeout /t 3
@cd ../batchs
:end
