import io

class BytesFIFO(object):
    """
    A FIFO that can store a fixed number of bytes.
    https://github.com/hbock/byte-fifo/blob/master/fifo.py
    """
    
    def __init__(self, init_size):
        """ Create a FIFO of ``init_size`` bytes. """
        self._buffer = io.BytesIO(b"\x00"*init_size)
        self._size = init_size
        self._filled = 0
        self._read_ptr = 0
        self._write_ptr = 0

    def read(self, size=-1):
        """
        Read at most ``size`` bytes from the FIFO.
        
        If less than ``size`` bytes are available, or ``size`` is negative,
        return all remaining bytes.
        """
        if size < 0:
            size = self._filled
        
        # Go to read pointer
        self._buffer.seek(self._read_ptr)

        # Figure out how many bytes we can really read
        size = min(size, self._filled)
        contig = self._size - self._read_ptr
        contig_read = min(contig, size)

        ret =  self._buffer.read(contig_read)
        self._read_ptr += contig_read
        if contig_read < size:
            leftover_size = size - contig_read
            self._buffer.seek(0)
            ret += self._buffer.read(leftover_size)
            self._read_ptr = leftover_size

        self._filled -= size
        
        return ret

    def write(self, data):
        """
        Write as many bytes of ``data`` as are free in the FIFO.
        
        If less than ``len(data)`` bytes are free, write as many as can be written.
        Returns the number of bytes written.
        """
        free = self.free()
        write_size = min(len(data), free)

        if write_size:
            contig = self._size - self._write_ptr
            contig_write = min(contig, write_size)
            # TODO: avoid 0 write
            # TODO: avoid copy
            # TODO: test performance of above
            self._buffer.seek(self._write_ptr)
            self._buffer.write(data[:contig_write])
            self._write_ptr += contig_write

            if contig < write_size:
                self._buffer.seek(0)
                self._buffer.write(data[contig_write:write_size])
                #self._buffer.write(buffer(data, contig_write, write_size - contig_write))
                self._write_ptr = write_size - contig_write

        self._filled += write_size

        return write_size

    def flush(self):
        """ Flush all data from the FIFO. """
        self._filled = 0
        self._read_ptr = 0
        self._write_ptr = 0
        
    def empty(self):
        """ Return ```True``` if FIFO is empty. """
        return self._filled == 0

    def full(self):
        """ Return ``True`` if FIFO is full. """
        return self._filled == self._size

    def free(self):
        """ Return the number of bytes that can be written to the FIFO. """
        return self._size - self._filled

    def capacity(self):
        """ Return the total space allocated for this FIFO. """
        return self._size

    def __len__(self):
        """ Return the amount of data filled in FIFO """
        return self._filled

    def __nonzero__(self):
        """ Return ```True``` if the FIFO is not empty. """
        return self._filled > 0

    def resize(self, new_size):
        """
        Resize FIFO to contain ``new_size`` bytes. If FIFO currently has
        more than ``new_size`` bytes filled, :exc:`ValueError` is raised.
        If ``new_size`` is less than 1, :exc:`ValueError` is raised.

        If ``new_size`` is smaller than the current size, the internal
        buffer is not contracted (yet).
        """
        if new_size < 1:
            raise ValueError("Cannot resize to zero or less bytes.")

        if new_size < self._filled:
            raise ValueError("Cannot contract FIFO to less than {} bytes, "
                             "or data will be lost.".format(self._filled))

        # original data is non-contiguous. we need to copy old data,
        # re-write to the beginning of the buffer, and re-sync
        # the read and write pointers.
        if self._read_ptr >= self._write_ptr:
            old_data = self.read(self._filled)
            self._buffer.seek(0)
            self._buffer.write(old_data)
            self._filled = len(old_data)
            self._read_ptr = 0
            self._write_ptr = self._filled
        
        self._size = new_size