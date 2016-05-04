/*
 *  IP checksumming functions.
 *  (c) 2008 Gerd Hoffmann <kraxel@redhat.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; under version 2 or later of the License.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, see <http://www.gnu.org/licenses/>.
 */

#include "qemu/osdep.h"
#include "qemu-common.h"
#include "net/checksum.h"
#include "net/eth.h"

uint32_t net_checksum_add_cont(int len, uint8_t *buf, int seq)
{
    uint32_t sum = 0;
    int i;

    for (i = seq; i < seq + len; i++) {
        if (i & 1) {
            sum += (uint32_t)buf[i - seq];
        } else {
            sum += (uint32_t)buf[i - seq] << 8;
        }
    }
    return sum;
}

uint16_t net_checksum_finish(uint32_t sum)
{
    while (sum>>16)
	sum = (sum & 0xFFFF)+(sum >> 16);
    return ~sum;
}

uint16_t net_checksum_tcpudp(uint16_t length, uint16_t proto,
                             uint8_t *addrs, uint8_t *buf)
{
    uint32_t sum = 0;

    sum += net_checksum_add(length, buf);         // payload
    sum += net_checksum_add(8, addrs);            // src + dst address
    sum += proto + length;                        // protocol & length
    return net_checksum_finish(sum);
}

void net_checksum_calculate(uint8_t *data, int length)
{
    int plen;
    struct ip_header *ip;

    /* Ensure we have at least a Eth header */
    if (length < sizeof(struct eth_header)) {
        return;
    }

    /* Now check we have an IP header (with an optonnal VLAN header */
    if (length < eth_get_l2_hdr_length(data) + sizeof(struct ip_header)) {
        return;
    }

    ip = PKT_GET_IP_HDR(data);

    if (IP_HEADER_VERSION(ip) != IP_HEADER_VERSION_4) {
	return; /* not IPv4 */
    }

    /* Last, check that we have enough data for the IP frame */
    if (length < eth_get_l2_hdr_length(data) + be16_to_cpu(ip->ip_len)) {
        return;
    }

    plen  = be16_to_cpu(ip->ip_len) - IP_HDR_GET_LEN(ip);

    switch (ip->ip_p) {
    case IP_PROTO_TCP:
        {
            uint16_t csum;
            tcp_header *tcp = (tcp_header *)(ip + 1);

            if (plen < sizeof(tcp_header)) {
                return;
            }

            tcp->th_sum = 0;

            csum = net_checksum_tcpudp(plen, ip->ip_p,
                                       (uint8_t *)&ip->ip_src,
                                       (uint8_t *)tcp);

            tcp->th_sum = cpu_to_be16(csum);
        }
	break;
    case IP_PROTO_UDP:
        {
            uint16_t csum;
            udp_header *udp = (udp_header *)(ip + 1);

            if (plen < sizeof(udp_header)) {
                return;
            }

            udp->uh_sum = 0;

            csum = net_checksum_tcpudp(plen, ip->ip_p,
                                       (uint8_t *)&ip->ip_src,
                                       (uint8_t *)udp);

            udp->uh_sum = cpu_to_be16(csum);
        }
	break;
    default:
        /* Can't handle any other protocol */
	return;
    }
}

uint32_t
net_checksum_add_iov(const struct iovec *iov, const unsigned int iov_cnt,
                     uint32_t iov_off, uint32_t size)
{
    size_t iovec_off, buf_off;
    unsigned int i;
    uint32_t res = 0;
    uint32_t seq = 0;

    iovec_off = 0;
    buf_off = 0;
    for (i = 0; i < iov_cnt && size; i++) {
        if (iov_off < (iovec_off + iov[i].iov_len)) {
            size_t len = MIN((iovec_off + iov[i].iov_len) - iov_off , size);
            void *chunk_buf = iov[i].iov_base + (iov_off - iovec_off);

            res += net_checksum_add_cont(len, chunk_buf, seq);
            seq += len;

            buf_off += len;
            iov_off += len;
            size -= len;
        }
        iovec_off += iov[i].iov_len;
    }
    return res;
}
