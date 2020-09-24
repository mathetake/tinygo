// +build wasm

package runtime

import (
	"unsafe"
)

// Implements __wasi_iovec_t.
type Wasi_iovec_t struct {
	Buf    unsafe.Pointer
	BufLen uint
}

//go:wasm-module wasi_unstable
//export fd_write
func Fd_write(id uint32, iovs *Wasi_iovec_t, iovs_len uint, nwritten *uint) (errno uint)

// Using global variables to avoid heap allocation.
var (
	putcharBuf   = byte(0)
	putcharIOVec = Wasi_iovec_t{
		Buf:    unsafe.Pointer(&putcharBuf),
		BufLen: 1,
	}
)

func postinit() {}

func putchar(c byte) {
	// write to stdout
	const stdout = 1
	var nwritten uint
	putcharBuf = c
	Fd_write(stdout, &putcharIOVec, 1, &nwritten)
}

// Abort executes the wasm 'unreachable' instruction.
func abort() {
	trap()
}

// TinyGo does not yet support any form of parallelism on WebAssembly, so these
// can be left empty.

//go:linkname procPin sync/atomic.runtime_procPin
func procPin() {
}

//go:linkname procUnpin sync/atomic.runtime_procUnpin
func procUnpin() {
}
