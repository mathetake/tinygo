// +build wasm,wasi

package os

import (
	"unsafe"

	"runtime"
)

// Stdin, Stdout, and Stderr are open Files pointing to the standard input,
// standard output, and standard error file descriptors.
var (
	Stdin  = &File{stdioFileHandle(0), "/dev/stdin"}
	Stdout = &File{stdioFileHandle(1), "/dev/stdout"}
	Stderr = &File{stdioFileHandle(2), "/dev/stderr"}
)

// isOS indicates whether we're running on a real operating system with
// filesystem support.
const isOS = false

// stdioFileHandle represents one of stdin, stdout, or stderr depending on the
// number. It implements the FileHandle interface.
type stdioFileHandle uint8

// Read is unsupported on this system.
func (f stdioFileHandle) Read(b []byte) (n int, err error) {
	return 0, ErrUnsupported
}

// Write writes len(b) bytes to the output. It returns the number of bytes
// written or an error if this file is not stdout or stderr.
func (f stdioFileHandle) Write(b []byte) (n int, err error) {
	vec := runtime.Wasi_iovec_t{
		Buf:    unsafe.Pointer(&b[0]),
		BufLen: uint(len(b)),
	}
	var nwritten uint
	errno := runtime.Fd_write(uint32(f), &vec, 1, &nwritten)
	if errno != 0 {
		return 0, ErrInvalid
	}
	return int(nwritten), nil
}

// Close is unsupported on this system.
func (f stdioFileHandle) Close() error {
	return ErrUnsupported
}
