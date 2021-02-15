package net

import (
	"context"
	"encoding/json"
	"github.com/pkg/errors"
	"golang.org/x/net/proxy"
	"io/ioutil"
	"math/rand"
	"net"
	"net/http"
	"net/url"
	"time"
)

var userAgents = []string{
	"Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/88.0.4324.150 Safari/537.36",
	"Mozilla/5.0 (iPhone; CPU iPhone OS 14_4 like Mac OS X) AppleWebKit/605.1.15 (KHTML, like Gecko) CriOS/87.0.4280.77 Mobile/15E148 Safari/604.1",
	"Mozilla/5.0 (Windows NT 10.0; Win64; x64; rv:85.0) Gecko/20100101 Firefox/85.0",
	"Mozilla/5.0 (X11; Ubuntu; Linux x86_64; rv:85.0) Gecko/20100101 Firefox/85.0",
	"Mozilla/5.0 (Macintosh; Intel Mac OS X 11_2_1) AppleWebKit/605.1.15 (KHTML, like Gecko) Version/14.0.2 Safari/605.1.15",
	"Mozilla/5.0 (Android 11; Mobile; rv:68.0) Gecko/68.0 Firefox/85.0",
}

type Client struct {
	client *http.Client
}

func randomUserAgent() string {
	return userAgents[rand.Intn(len(userAgents))]
}

func NewProxiedClient() *Client {
	proxyDialer := proxy.FromEnvironmentUsing(&net.Dialer{
		Timeout:   30 * time.Second,
		KeepAlive: 30 * time.Second,
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

func (c *Client) Get(uri string) ([]byte, error) {
	req, err := http.NewRequest("GET", uri, nil)
	if err != nil {
		return nil, err
	}

	if domain := baseDomain(uri); domain != "" {
		req.Header.Set("Referer", domain)
	}
	req.Header.Set("User-Agent", randomUserAgent())

	resp, err := c.client.Do(req)
	if err != nil {
		return nil, errors.Wrap(err, "airkaz get failed")
	}
	defer resp.Body.Close()

	return ioutil.ReadAll(resp.Body)
}

func (c *Client) GetJSON(url string, buf interface{}) error {
	data, err := c.Get(url)
	if err != nil {
		err = json.Unmarshal(data, buf)
	}
	return err
}
