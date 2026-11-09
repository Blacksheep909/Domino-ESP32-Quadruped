; Custom steps for the Windows installer (electron-builder NSIS include).
;
; Studio used to name its shortcuts "Domino Quadruped Studio v<version>". An
; update keeps the existing shortcut, so the Start menu kept the first
; version's name (v0.4.8) while newer versions were installed. The shortcut
; is now just "Domino Quadruped Studio"; this removes the old versioned ones
; and makes sure the current shortcut exists.

!macro dominoReplaceVersionedShortcuts FOLDER NEWLINK
  ClearErrors
  FindFirst $R8 $R9 "${FOLDER}\Domino Quadruped Studio v*.lnk"
  ${DoWhile} $R9 != ""
    Delete "${FOLDER}\$R9"
    FindNext $R8 $R9
  ${Loop}
  FindClose $R8
  ${IfNot} ${FileExists} "${NEWLINK}"
    CreateShortCut "${NEWLINK}" "$appExe" "" "$appExe" 0 "" "" "${APP_DESCRIPTION}"
    WinShell::SetLnkAUMI "${NEWLINK}" "${APP_ID}"
  ${EndIf}
  ClearErrors
!macroend

!macro customInstall
  ; Only touch shortcut folders that held a versioned shortcut, so a shortcut
  ; the user deleted on purpose is not brought back.
  ClearErrors
  FindFirst $R6 $R7 "$SMPROGRAMS\Domino Quadruped Studio v*.lnk"
  FindClose $R6
  ${If} $R7 != ""
    !insertmacro dominoReplaceVersionedShortcuts "$SMPROGRAMS" "$newStartMenuLink"
  ${EndIf}
  ClearErrors
  FindFirst $R6 $R7 "$DESKTOP\Domino Quadruped Studio v*.lnk"
  FindClose $R6
  ${If} $R7 != ""
    !insertmacro dominoReplaceVersionedShortcuts "$DESKTOP" "$newDesktopLink"
  ${EndIf}
  ClearErrors
  System::Call 'Shell32::SHChangeNotify(i 0x8000000, i 0, i 0, i 0)'
!macroend
