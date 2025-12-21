package net

import (
	"context"
	"encoding/json"
	"io"
	"math/rand"
	"net"
	"net/http"
	"net/url"
	"time"

	"github.com/hg/airmon/logger"
	"github.com/pkg/errors"
	"go.uber.org/zap"
	"golang.org/x/net/proxy"
)

var userAgents = []string{
	"Mozilla/5.0 (Macintosh; Intel Mac OS X 10_15_7) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/143.0.0.0 Safari/537.36 Edg/143.0.3650.96",
	"Mozilla/5.0 (Macintosh; Intel Mac OS X 10_15_7) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/143.0.0.0 Safari/537.36",
	"Mozilla/5.0 (Macintosh; Intel Mac OS X 15.7; rv:146.0) Gecko/20100101 Firefox/146.0",
	"Mozilla/5.0 (Macintosh; Intel Mac OS X 15_7_3) AppleWebKit/605.1.15 (KHTML, like Gecko) Version/26.0 Safari/605.1.15",
	"Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/143.0.0.0 Safari/537.36 Edg/143.0.3650.96",
	"Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/143.0.0.0 Safari/537.36",
	"Mozilla/5.0 (Windows NT 10.0; Win64; x64; rv:146.0) Gecko/20100101 Firefox/146.0",
	"Mozilla/5.0 (X11; Linux x86_64; rv:146.0) Gecko/20100101 Firefox/146.0",
}

var log = logger.Get(logger.Net)

type Client struct {
	client *http.Client
}

func randomUserAgent() string {
	return userAgents[rand.Intn(len(userAgents))]
}

func NewProxiedClient() *Client {
	proxyDialer := proxy.FromEnvironmentUsing(&net.Dialer{
		Timeout:   180 * time.Second,
		KeepAlive: 180 * time.Second,
	})
	return &Client{
		client: &http.Client{
			Transport: &http.Transport{
				DialContext: func(ctx context.Context, network, addr string) (net.Conn, error) {
					return proxyDialer.Dial(network, addr)
				},
				MaxIdleConns:        1,
				IdleConnTimeout:     1 * time.Minute,
				TLSHandshakeTimeout: 10 * time.Second,
			},
		},
	}
}

func baseDomain(fullUrl string) string {
	if parsed, err := url.Parse(fullUrl); err != nil {
		return ""
	} else {
		parsed.Opaque = ""
		parsed.Path = ""
		parsed.RawQuery = ""
		parsed.ForceQuery = false
		parsed.Fragment = ""
		parsed.RawFragment = ""
		return parsed.String()
	}
}

func (c *Client) get(uri string, accept string) ([]byte, error) {
	req, err := http.NewRequest("GET", uri, nil)
	if err != nil {
		return nil, err
	}

	if domain := baseDomain(uri); domain != "" {
		req.Header.Set("Referer", domain)
		req.Header.Set("Host", domain)
	}
	req.Header.Set("Accept", accept)
	req.Header.Set("User-Agent", randomUserAgent())
	req.Header.Set("Accept-Language", "ru,en;q=0.5")
	req.Header.Set("Cache-Control", "no-cache")
	req.Header.Set("Sec-Fetch-Dest", "document")
	req.Header.Set("Sec-Fetch-Mode", "navigate")
	req.Header.Set("Sec-Fetch-Site", "same-origin")
	req.Header.Set("Sec-Fetch-User", "?1")

	resp, err := c.client.Do(req)
	if err != nil {
		log.Error("get failed", zap.Error(err))
		return nil, errors.Wrap(err, "net request failed")
	}
	defer resp.Body.Close()

	return io.ReadAll(resp.Body)
}

func (c *Client) Get(url string) ([]byte, error) {
	const accept = "text/html,application/xhtml+xml,application/xml;q=0.9,*/*;q=0.8"
	return c.get(url, accept)
}

func (c *Client) GetJSON(url string, buf interface{}) error {
	const accept = "application/json,text/json;q=0.9"
	data, err := c.get(url, accept)
	if err == nil {
		err = json.Unmarshal(data, buf)
	}
	if err != nil {
		log.Error("getJson failed",
			zap.String("url", url),
			zap.Error(err))
	}
	return err
}
