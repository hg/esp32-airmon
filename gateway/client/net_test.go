package client

import "testing"

func TestBaseDomain(t *testing.T) {
	tests := []struct {
		in, want string
	}{
		{in: "http://example.com/foo/bar?x=1#section", want: "http://example.com"},
		{in: "https://example.com/", want: "https://example.com"},
		{in: "https://example.com:8443/api/v1?debug=true", want: "https://example.com:8443"},
		{in: "http://example.com", want: "http://example.com"},
		{in: "https://api.dev.example.com/v2/users", want: "https://api.dev.example.com"},
		{in: "https://user:pass@example.com/private", want: "https://user:pass@example.com"},
		{in: "://not a url", want: ""},
		{in: "", want: ""},
	}

	for _, tt := range tests {
		t.Run(tt.in, func(t *testing.T) {
			got := baseDomain(tt.in)
			if got != tt.want {
				t.Fatalf("baseDomain(%q) = %q, want %q", tt.in, got, tt.want)
			}
		})
	}
}
