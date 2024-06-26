/*
 * kl_fs_common.cpp
 *
 *  Created on: 30 ���. 2016 �.
 *      Author: Kreyl
 */

#include "kl_fs_utils.h"
#include "shell.h"

// Variables
FILINFO file_info;
DIR Dir;
FIL common_file;
static char istr[SD_STRING_SZ];

#if 1 // ============================== Common =================================
retv TryOpenFileRead(const char *Filename, FIL *PFile) {
//    Printf("%S: %S; %X\r", __FUNCTION__, Filename, PFile);
    FRESULT rslt = f_open(PFile, Filename, FA_READ);
    if(rslt == FR_OK) {
        // Check if zero file
        if(f_size(PFile) == 0) {
            f_close(PFile);
            Printf("Empty file %S\r", Filename);
            return retv::Empty;
        }
        return retv::Ok;
    }
    else {
        if (rslt == FR_NO_FILE) Printf("%S: not found\r", Filename);
        else Printf("OpenFile error: %u\r", rslt);
        return retv::Fail;
    }
}

retv TryOpenFileRewrite(const char *Filename, FIL *PFile) {
    FRESULT rslt = f_open(PFile, Filename, FA_WRITE+FA_CREATE_ALWAYS);
    if(rslt == FR_OK) return retv::Ok;
    else {
        Printf("%S open error: %u\r", Filename, rslt);
        return retv::Fail;
    }
}

void CloseFile(FIL *PFile) {
    f_close(PFile);
}

retv CheckFileNotEmpty(FIL *PFile) {
    if(f_size(PFile) == 0) {
        Printf("Empty file\r");
        return retv::Empty;
    }
    else return retv::Ok;
}

retv TryRead(FIL *PFile, void *Ptr, uint32_t Sz) {
    uint32_t ReadSz=0;
    uint8_t r = f_read(PFile, Ptr, Sz, &ReadSz);
    return (r == FR_OK and ReadSz == Sz)? retv::Ok : retv::Fail;
}

retv ReadLine(FIL *PFile, char* S, uint32_t MaxLen) {
    uint32_t Len = 0, Rcv;
    char c, str[2];
    while(Len < MaxLen-1) {
        if(f_read(PFile, str, 1, &Rcv) != FR_OK) return retv::Fail;
        if(Rcv != 1) return retv::EndOfFile;
        c = str[0];
        if(c == '\r' or c == '\n') {    // End of line
            *S = '\0';
            return retv::Ok;
        }
        else {
            *S++ = c;
            Len++;
        }
    } // while
    *S = '\0';
    return retv::Ok;
}

bool DirExists(const char* DirName) {
    FRESULT Rslt = f_opendir(&Dir, DirName);
    return (Rslt == FR_OK);
}

bool DirExistsAndContains(const char* DirName, const char* Extension) {
    if(f_opendir(&Dir, DirName) == FR_OK) {
        while(true) {
            // Empty names before reading
            *file_info.fname = 0;
#if _USE_LFN
            *file_info.altname = 0;
#endif
            if(f_readdir(&Dir, &file_info) != FR_OK) return false;
            if((file_info.fname[0] == 0)
#if _USE_LFN
                    and (file_info.altname[0] == 0)
#endif
            ) return false;   // No files left
            else { // Filename ok, check if not dir
                if(!(file_info.fattrib & AM_DIR)) {
                    // Check the ext
#if _USE_LFN
                    char *FName = (file_info.fname[0] == 0)? file_info.altname : file_info.fname;
#else
                    char *FName = file_info.fname;
#endif
                    uint32_t Len = strlen(FName);
                    if(Len > 4) {
                        if(strncasecmp(&FName[Len-3], Extension, 3) == 0) return true;
                    } // if Len>4
                } // if not dir
            } // Filename ok
        } // while
    }
    else return false;
}

retv CountFilesInDir(const char* DirName, const char* Extension, uint32_t *PCnt) {
    *PCnt = 0;
    FRESULT Rslt = f_opendir(&Dir, DirName);
//    Printf("f_opendir %S: %u\r", DirName, Rslt);
    if(Rslt != FR_OK) return retv::Fail;
    while(true) {
        // Empty names before reading
        *file_info.fname = 0;
#if _USE_LFN
        *file_info.altname = 0;
#endif

        Rslt = f_readdir(&Dir, &file_info);
        if(Rslt != FR_OK) return retv::Fail;
        if((file_info.fname[0] == 0)
#if _USE_LFN
                    and (file_info.altname[0] == 0)
#endif
        ) return retv::Ok;   // No files left
        else { // Filename ok, check if not dir
            if(!(file_info.fattrib & AM_DIR)) {
                // Check Ext
#if _USE_LFN
                char *FName = (file_info.fname[0] == 0)? file_info.altname : file_info.fname;
#else
                char *FName = file_info.fname;
#endif
//                Printf("%S\r", FName);
                uint32_t Len = strlen(FName);
                if(Len > 4) {
                    if(strncasecmp(&FName[Len-3], Extension, 3) == 0) (*PCnt)++;
                } // if Len>4
            } // if not dir
        } // Filename ok
    }
    return retv::Ok;
}

retv CountDirsStartingWith(const char* Path, const char* DirNameStart, uint32_t *PCnt) {
    FRESULT Rslt = f_opendir(&Dir, Path);
    if(Rslt != FR_OK) return retv::Fail;
    *PCnt = 0;
    int Len = strlen(DirNameStart);
    while(true) {
        // Empty names before reading
        *file_info.fname = 0;
#if _USE_LFN
        *file_info.altname = 0;
#endif
        Rslt = f_readdir(&Dir, &file_info);
        if(Rslt != FR_OK) return retv::Fail;
        if((file_info.fname[0] == 0)
#if _USE_LFN
                    and (file_info.altname[0] == 0)
#endif
        ) return retv::Ok;   // Nothing left
        else { // Filename ok, check if dir
            if(file_info.fattrib & AM_DIR) {
                // Check if starts with DirNameStart
#if _USE_LFN
                char *FName = (file_info.fname[0] == 0)? file_info.altname : file_info.fname;
#else
                char *FName = file_info.fname;
#endif
//                Printf("%S\r", FName);
                if(strncasecmp(FName, DirNameStart, Len) == 0) (*PCnt)++;
            } // if dir
        } // Filename ok
    }
    return retv::Ok;
}
#endif

namespace ini { // =================== ini file operations =====================
void WriteSection(FIL *PFile, const char *ASection) {
    f_printf(PFile, "[%S]\r\n", ASection);
}
void WriteString(FIL *PFile, const char *AKey, char *AValue) {
    f_printf(PFile, "%S=%S\r\n", AKey, AValue);
}
void WriteInt32(FIL *PFile, const char *AKey, const int32_t AValue) {
    f_printf(PFile, "%S=%D\r\n", AKey, AValue);
}
void WriteNewline(FIL *PFile) {
    f_putc('\r', PFile);
    f_putc('\n', PFile);
}
void WriteComment(FIL *PFile, const char* Str) {
    f_printf(PFile, "# %S\r\n", Str);
}


static inline char* skipleading(char *S) {
    while (*S != '\0' && *S <= ' ') S++;
    return S;
}
static inline char* skiptrailing(char *S, const char *base) {
    while ((S > base) && (*(S-1) <= ' ')) S--;
    return S;
}
static inline char* striptrailing(char *S) {
    char *ptr = skiptrailing(strchr(S, '\0'), S);
    *ptr='\0';
    return S;
}

retv ReadString(const char *AFileName, const char *ASection, const char *AKey, char **PPOutput) {
    FRESULT rslt;
//    Printf("%S %S %S %S\r", __FUNCTION__, AFileName, ASection, AKey);
    // Open file
    rslt = f_open(&common_file, AFileName, FA_READ+FA_OPEN_EXISTING);
    if(rslt != FR_OK) {
        if (rslt == FR_NO_FILE) Printf("%S: not found\r", AFileName);
        else Printf("%S: openFile error: %u\r", AFileName, rslt);
        return retv::Fail;
    }
    // Check if zero file
    if(f_size(&common_file) == 0) {
        f_close(&common_file);
        Printf("Empty file\r");
        return retv::Fail;
    }
    // Move through file one line at a time until a section is matched or EOF.
    char *StartP, *EndP = nullptr;
    int32_t len = strlen(ASection);
    do {
        if(f_gets(istr, SD_STRING_SZ, &common_file) == nullptr) {
            Printf("iniNoSection %S\r", ASection);
            f_close(&common_file);
            return retv::Fail;
        }
        StartP = skipleading(istr);
        if((*StartP != '[') or (*StartP == ';') or (*StartP == '#')) continue;
        EndP = strchr(StartP, ']');
        if((EndP == NULL) or ((int32_t)(EndP-StartP-1) != len)) continue;
    } while (strncmp(StartP+1, ASection, len) != 0);

    // Section found, find the key
    len = strlen(AKey);
    do {
        if(!f_gets(istr, SD_STRING_SZ, &common_file) or *(StartP = skipleading(istr)) == '[') {
            Printf("iniNoKey %S\r", AKey);
            f_close(&common_file);
            return retv::Fail;
        }
        StartP = skipleading(istr);
        if((*StartP == ';') or (*StartP == '#')) continue;
        EndP = strchr(StartP, '=');
        if(EndP == NULL) continue;
    } while(((int32_t)(skiptrailing(EndP, StartP)-StartP) != len or strncmp(StartP, AKey, len) != 0));
    f_close(&common_file);

    // Process Key's value
    StartP = skipleading(EndP + 1);
    // Remove a trailing comment
    uint8_t isstring = 0;
    for(EndP = StartP; (*EndP != '\0') and (((*EndP != ';') and (*EndP != '#')) or isstring) and ((uint32_t)(EndP - StartP) < SD_STRING_SZ); EndP++) {
        if (*EndP == '"') {
            if (*(EndP + 1) == '"') EndP++;     // skip "" (both quotes)
            else isstring = !isstring; // single quote, toggle isstring
        }
        else if (*EndP == '\\' && *(EndP + 1) == '"') EndP++; // skip \" (both quotes)
    } // for
    *EndP = '\0';   // Terminate at a comment
    striptrailing(StartP);
    *PPOutput = StartP;
    return retv::Ok;
}

retv ReadStringTo(const char *AFileName, const char *ASection, const char *AKey, char *POutput, uint32_t MaxLen) {
    char *S;
    if(ReadString(AFileName, ASection, AKey, &S) == retv::Ok) {
        // Copy what was read
        if(strlen(S) > (MaxLen-1)) {
            strncpy(POutput, S, (MaxLen-1));
            POutput[MaxLen-1] = 0;  // terminate string
        }
        else strcpy(POutput, S);
        return retv::Ok;
    }
    else return retv::Fail;
}

retv HexToUint(char *S, uint8_t AMaxLength, uint32_t *AOutput) {
    *AOutput = 0;
    char c;
    uint8_t b=0;
    for(uint8_t i=0; i<AMaxLength; i++) {
        c = *S++;
        if (c == 0) return retv::Ok;    // end of string
        // Shift result
        *AOutput <<= 4;
        // Get next digit
        if     ((c >= '0') && (c <= '9')) b = c-'0';
        else if((c >= 'A') && (c <= 'F')) b = c-'A'+10;
        else if((c >= 'a') && (c <= 'f')) b = c-'a'+10;
        else return retv::Fail;  // not a hex digit
        *AOutput += b;
    }
    return retv::Ok;
}

retv ReadColor (const char *AFileName, const char *ASection, const char *AKey, Color_t *AOutput) {
    char *S;
    if(ReadString(AFileName, ASection, AKey, &S) == retv::Ok) {
        if(strlen(S) != 6) return retv::BadValue;
        uint32_t N=0;
        if(HexToUint(&S[0], 2, &N) != retv::Ok) return retv::Fail;
        AOutput->R = N;
        if(HexToUint(&S[2], 2, &N) != retv::Ok) return retv::Fail;
        AOutput->G = N;
        if(HexToUint(&S[4], 2, &N) != retv::Ok) return retv::Fail;
        AOutput->B = N;
        return retv::Ok;
    }
    else return retv::Fail;
}

} // Namespace

namespace csv { // =================== csv file operations =====================
#define CSV_DELIMITERS  " ,;={}\t\r\n"
__unused static char *csvCurToken;

retv OpenFile(const char *AFileName) {
    return TryOpenFileRead(AFileName, &common_file);
}
void RewindFile() {
    f_lseek(&common_file, 0);
}
void CloseFile() {
    f_close(&common_file);
}

/* Skips empty and comment lines (starting with #).
 * Returns retv::Ok if not-a-comment line read, retvEndOfFile if eof
 */
retv ReadNextLine() {
    // Move through file until comments end
    while(true) {
        if(f_eof(&common_file) or f_gets(istr, SD_STRING_SZ, &common_file) == nullptr) {
//            Printf("csvNoMoreData\r");
            return retv::EndOfFile;
        }
        csvCurToken = strtok(istr, CSV_DELIMITERS);
        if(*csvCurToken == '#' or *csvCurToken == 0) continue; // Skip comments and empty lines
        else return retv::Ok;
    }
}

retv GetNextToken(char** POutput) {
    // First time, return csvCurToken, as it was set in ReadNextLine
    if(csvCurToken != nullptr) {
        *POutput = csvCurToken;
        csvCurToken = nullptr;
    }
    else *POutput = strtok(NULL, CSV_DELIMITERS);
    return (**POutput == '\0')? retv::Empty : retv::Ok;
}

retv GetNextCellString(char* POutput) {
    char *Token;
    if(GetNextToken(&Token) == retv::Ok) {
        // Skip leading quotes
        char *StartP = Token;
        while(*StartP == '"' and *StartP != '\0') StartP++;
        char *EndP = Token + strlen(Token) - 1;
        while(*EndP == '"' and EndP > Token) EndP--;
        int32_t Len = EndP - StartP + 1;
        if(Len > 0) strncpy(POutput, StartP, Len);
        else *POutput = '\0';
        return retv::Ok;
    }
    else return retv::Fail;
}

retv FindFirstCell(const char* Name) {
    while(true) {
        if(ReadNextLine() != retv::Ok) break;
//        Printf("Token: %S\r", csvCurToken);
        if(strcasecmp(csvCurToken, Name) == 0) {
            csvCurToken = strtok(NULL, CSV_DELIMITERS);
            return retv::Ok;
        }
    }
    return retv::NotFound;
}

retv GetNextCell(float *POutput) {
    char *Token;
    if(GetNextToken(&Token) == retv::Ok) {
        char *p;
        *POutput = strtof(Token, &p);
        if(*p == '\0') return retv::Ok;
        else return retv::NotANumber;
    }
    else return retv::Empty;
}

retv TryLoadParam(char* Token, const char* Name, float *Ptr) {
    if(strcasecmp(Token, Name) == 0) {
        if(csv::GetNextCell(Ptr) == retv::Ok){
//            Printf("  %S: %f\r", Name, *Ptr);
            return retv::Ok;
        }
        else Printf("%S load fail\r", Name);
    }
    return retv::Fail;
}

retv TryLoadString(char* Token, const char* Name, char *Dst, uint32_t MaxLen) {
    if(strcasecmp(Token, Name) == 0) {
        *Dst = 0;   // Empty Dst
        char *Cell;
        if(GetNextToken(&Cell) == retv::Ok) {
            uint32_t Len = strlen(Cell);
            if(Len < MaxLen) strcpy(Dst, Cell);
            else {
                strncpy(Dst, Cell, MaxLen-1);
                Dst[MaxLen-1] = 0;
            }
        }
        return retv::Ok;
    }
    return retv::Fail;
}

} // namespace
