// config.example.h — 部署配置模板（提交到 git）
//
// 用法：复制本文件为同目录下的 config.h，再填入你自己的值。
//   cp config.example.h config.h
// config.h 已被 .gitignore 忽略，不会泄露你的服务器信息。
#pragma once

#define CATM_APN     "your.apn"                        // SIM 卡 APN，如 "povo.jp"
#define SERVER_HOST  "your-server.example.com"         // 服务器域名（TLS SNI / Host）
#define SERVER_PORT  443                               // 服务器端口
#define SERVER_BASE  "https://your-server.example.com"
#define PATH_APRS    "/a"                              // 接收 APRS 点位的后端路径
