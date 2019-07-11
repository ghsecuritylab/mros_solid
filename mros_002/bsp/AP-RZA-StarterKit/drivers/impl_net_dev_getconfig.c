#include "impl_net_dev.h"

/*
 * 静的IPアドレスを利用する場合には STATIC_IP_ADDRの
 * 定義を有効にした状態でNetIfConfig_t型の構造体に
 * 設定を記述し、関数 IMPL_NET_DEV_get_netconfigの
 * 戻り値として先頭アドレスを返してください。
 *
 * 関数の戻り値をNULLにするとプロトコルスタックは
 * DHCPによる自動IPアドレス取得動作を行います。
 *
 */

#define STATIC_IP_ADDR		/* 静的IPを使用 */

#ifdef STATIC_IP_ADDR
const NetIfConfig_t	_network_config =
{
		"192.168.11.100",	// Local IP address
		"255.255.255.0",	// Subnet mask
		"192.168.11.1",		// Gateway IP address
		NO_DNS_SERVER_LIST()// list of DNS servers
};

/*
 * DNSサーバーのアドレスを指定する場合には以下の例の様に
 * サーバーのアドレス文字列へのポインタを格納する配列を
 * 別の変数として定義し、マクロ SET_DNS_SERVER_LIST(変数名)で
 * NetIfConfig_t構造体のメンバ変数 dns_server_listを初期化します。
 * (サーバーが無い場合にはマクロNO_DNS_SERVER_LIST()を使用します）
 *
 * const	char	*ns_list[] = {"192.168.0.1", "192.168.10.1"};
 * const NetIfConfig_t	_network_static_with_dns =
 * {
 *        "192.168.0.100",	// Local IP address
 *        "255.255.255.0",	// Subnet mask
 *        "192.168.0.1",		// Gateway IP address
 *        SET_DNS_SERVER_LIST(ns_list) // list of DNS servers
 * };
 *
 */

#else

/*
 * DHCP使用時にDNSサーバーのアドレスを指定するには
 * ローカルIPアドレスにNULLを指定（DHCP動作）した上で
 * DNSサーバーを指定します。
 *
 * ただし、DHCPサーバーからのレスポンスにDNSサーバーの
 * 情報が含まれる場合にはここで指定したDNSサーバーの
 * アドレスは上書きされます。
 * const	char	*ns_list[] = {"192.168.0.1"};
 * const NetIfConfig_t	_network_dhcp_with_dns = {
 *         NULL, 				// Local IP address
 *         NULL,				// Subnet mask
 *         NULL,				// Gateway IP address
 *         SET_DNS_SERVER_LIST(ns_list) * // list of DNS servers
 * };
 *
 */

#endif

const NetIfConfig_t *IMPL_NET_DEV_get_netconfig(void)
{
#ifdef STATIC_IP_ADDR
	return &_network_config;
#else
	return NULL;	/* DHCPを使用 */
#endif
}
