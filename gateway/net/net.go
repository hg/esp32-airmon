package net

import (
	"context"
	"golang.org/x/net/proxy"
	"net"
	"net/http"
	"time"
)

func NewProxiedClient() *http.Client {
	proxyDialer := proxy.FromEnvironmentUsing(&net.Dialer{
		Timeout:   30 * time.Second,
		KeepAlive: 30 * time.Second,
	})
	return &http.Client{
		Transport: &http.Transport{
			DialContext: func(ctx context.Context, network, addr string) (net.Conn, error) {
				return proxyDialer.Dial(network, addr)
			},
			MaxIdleConns:        1,
			IdleConnTimeout:     1 * time.Minute,
			TLSHandshakeTimeout: 10 * time.Second,
		},
	}
}
