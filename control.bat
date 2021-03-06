@echo off
REM -------------------------------------------------------
call :cmd_%1 %2 %3 %4 %5 %6 %7 %8 %9
exit 1
REM -------------------------------------------------------
:cmd_gen
    call :call_script "gen_project"
:cmd_vs
    call :call_script "run_project"
:cmd_add_unit
    call :call_any git submodule add --name "%2" "%3" "%1/%2"
REM -------------------------------------------------------
:call_script
    call "./build/scripts/%1.bat"
    exit 0
:call_any
    call %1 %2 %3 %4 %5 %6 %7 %8 %9
    exit 0
