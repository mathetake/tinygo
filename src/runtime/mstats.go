package runtime

type MemStats struct {
	PauseNs [256]uint64
}

//go:linkname readGCStats runtime/debug.readGCStats
func readGCStats(pauses *[]uint64) {
}
