#include <syslog.h>
#include <stdio.h>
#include <string.h>
/* lwIP headers */
#include "lwip/api.h"
#include "lwip/tcpip.h"
#include "lwip/memp.h"
#include "lwip/tcp.h"
#include "lwip/udp.h"
#include "netif/etharp.h"
#include "lwip/dhcp.h"
#include "lwip/ip_addr.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"

void root_task(){

	int sockfd;

	int new_sockfd;

	unsigned int writer_len;

	struct sockaddr_in reader_addr;

	struct sockaddr_in writer_addr;

	char buf[100];
	/* ソケットの生成 */

	if ((sockfd = socket(PF_INET, SOCK_STREAM, 0)) < 0)
	{
		//ソケット生成が失敗した場合は-1が返される
		perror("reader: socket");

		exit(1);
	}

	/* 通信ポート・アドレスの設定 */

	bzero((char *)&reader_addr, sizeof(reader_addr));

	reader_addr.sin_family = PF_INET;	//プロトコルファミリ

	reader_addr.sin_addr.s_addr = lwip_htonl(INADDR_ANY);	//INADDR_ANY=0.0.0.0　どのアドレスからの接続でも受け入れるように待ち受ける

	reader_addr.sin_port = lwip_htons(100);					//ポート番号

	/* ソケットにアドレスやポート番号を結びつける */

	if (bind(sockfd, (struct sockaddr *)&reader_addr, sizeof(reader_addr)) < 0)	//引数は順にソケット番号、アドレスやポート番号を格納した構造体へのポインタ、第2引数のサイズ
	{

		perror("reader: bind");

		exit(1);
	}

	/* コネクト要求をいくつまで待つかを設定 */

	if (listen(sockfd, 5) < 0)
	{

		perror("reader: listen");

		close(sockfd);

		exit(1);
	}

	/* コネクト要求を待つ。accept()を実行した時点でサーバー側プログラムは一旦停止する */

	if ((new_sockfd = accept(sockfd, (struct sockaddr *)&writer_addr,&writer_len)) < 0)
	{
		/* accept()はクライアントからの接続要求が来ると同時に新しいソケットを生成する。
		   新しいソケットはクライアントとの接続に使われるソケットで、もともとのソケットは次の接続要求の受付のために使われる
		　返り値は新しいソケットの番号になる。失敗したら-1を返す
		*/
		perror("reader: accept");

		exit(1);
	}
	recv(new_sockfd,buf,sizeof(char),0);
	syslog(LOG_INFO,"received data:%s\n",buf);
	close(sockfd); /* ソケットを閉鎖 */
}
/*
void simple_server(int sockfd)
{

	char buf[1];

	int buf_len;

	while ((buf_len = read(new_sockfd, buf, 1)) > 0)
	{

		write(1, buf, buf_len);
	}

	close(new_sockfd); // ソケットを閉鎖 
}
*/