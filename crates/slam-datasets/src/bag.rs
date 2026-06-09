//! Minimal ROS1 v2.0 bag reader with **index-driven chunk selection** (ADR 0008).
//!
//! Why not the `rosbag` crate: bag extraction is dominated by per-chunk bzip2
//! decompression, and the format's own index (`ChunkInfo` records at `index_pos`) says
//! exactly which chunks contain which connections. The crate decompresses every chunk
//! and exposes neither chunk skipping nor the raw compressed payloads needed for
//! parallel decompression. This reader parses the index first and only ever touches the
//! chunks that carry requested connections.
//!
//! Format (v2.0): a magic line, then length-prefixed records — `u32 header_len`, header
//! bytes (length-prefixed `name=value` fields), `u32 data_len`, data bytes. Records used
//! here: bag header (op 0x03, carries `index_pos`), chunk (0x05, compressed payload of
//! connection/message records), connection (0x07), message data (0x02), chunk info
//! (0x06). Everything else is skipped.

use std::collections::BTreeSet;
use std::fs::File;
use std::io::{Read, Seek, SeekFrom};
use std::path::Path;

use crate::BagError;

const MAGIC: &[u8] = b"#ROSBAG V2.0\n";

const OP_MESSAGE_DATA: u8 = 0x02;
const OP_BAG_HEADER: u8 = 0x03;
const OP_CHUNK: u8 = 0x05;
const OP_CHUNK_INFO: u8 = 0x06;
const OP_CONNECTION: u8 = 0x07;

/// One connection: a topic + message type under a numeric id.
#[derive(Debug, Clone)]
pub struct Connection {
    pub id: u32,
    pub topic: String,
    pub message_type: String,
}

/// Index entry for one chunk: where it lives and which connections it contains.
#[derive(Debug)]
struct ChunkEntry {
    pos: u64,
    connections: Vec<u32>,
}

/// An opened bag: connections + chunk index parsed, no chunk touched yet.
pub struct BagFile {
    file: File,
    connections: Vec<Connection>,
    chunks: Vec<ChunkEntry>,
}

// ---------------------------------------------------------------------------------------
// Wire-level helpers
// ---------------------------------------------------------------------------------------

/// Bounds-checked little-endian cursor over an in-memory buffer.
struct Cur<'a> {
    data: &'a [u8],
    pos: usize,
}

impl<'a> Cur<'a> {
    fn new(data: &'a [u8]) -> Self {
        Cur { data, pos: 0 }
    }

    fn remaining(&self) -> usize {
        self.data.len() - self.pos
    }

    fn take(&mut self, n: usize) -> Result<&'a [u8], BagError> {
        let end = self
            .pos
            .checked_add(n)
            .filter(|&e| e <= self.data.len())
            .ok_or(BagError::Format("record truncated"))?;
        let s = &self.data[self.pos..end];
        self.pos = end;
        Ok(s)
    }

    fn u32(&mut self) -> Result<u32, BagError> {
        let b = self.take(4)?;
        Ok(u32::from_le_bytes([b[0], b[1], b[2], b[3]]))
    }
}

/// A record header: `name=value` fields, values are raw bytes.
struct HeaderMap<'a>(Vec<(&'a [u8], &'a [u8])>);

impl<'a> HeaderMap<'a> {
    fn parse(bytes: &'a [u8]) -> Result<Self, BagError> {
        let mut c = Cur::new(bytes);
        let mut fields = Vec::new();
        while c.remaining() > 0 {
            let len = c.u32()? as usize;
            let field = c.take(len)?;
            let eq = field
                .iter()
                .position(|&b| b == b'=')
                .ok_or(BagError::Format("header field without '='"))?;
            fields.push((&field[..eq], &field[eq + 1..]));
        }
        Ok(HeaderMap(fields))
    }

    fn get(&self, name: &[u8]) -> Option<&'a [u8]> {
        self.0.iter().find(|(n, _)| *n == name).map(|(_, v)| *v)
    }

    fn op(&self) -> Result<u8, BagError> {
        match self.get(b"op") {
            Some([op]) => Ok(*op),
            _ => Err(BagError::Format("record without op field")),
        }
    }

    fn u32_field(&self, name: &'static [u8]) -> Result<u32, BagError> {
        match self.get(name) {
            Some(v) if v.len() == 4 => Ok(u32::from_le_bytes([v[0], v[1], v[2], v[3]])),
            _ => Err(BagError::Format("missing/short u32 header field")),
        }
    }

    fn u64_field(&self, name: &'static [u8]) -> Result<u64, BagError> {
        match self.get(name) {
            Some(v) if v.len() == 8 => Ok(u64::from_le_bytes([
                v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7],
            ])),
            _ => Err(BagError::Format("missing/short u64 header field")),
        }
    }

    fn str_field(&self, name: &'static [u8]) -> Result<&'a str, BagError> {
        std::str::from_utf8(
            self.get(name)
                .ok_or(BagError::Format("missing string header field"))?,
        )
        .map_err(|_| BagError::Format("non-UTF-8 header field"))
    }
}

/// Read one record (header bytes + data bytes) at the file's current position.
fn read_record_from_file(file: &mut File) -> Result<(Vec<u8>, Vec<u8>), BagError> {
    let mut len = [0u8; 4];
    file.read_exact(&mut len)?;
    let mut header = vec![0u8; u32::from_le_bytes(len) as usize];
    file.read_exact(&mut header)?;
    file.read_exact(&mut len)?;
    let mut data = vec![0u8; u32::from_le_bytes(len) as usize];
    file.read_exact(&mut data)?;
    Ok((header, data))
}

/// Read one record out of an in-memory buffer.
fn read_record<'a>(c: &mut Cur<'a>) -> Result<(HeaderMap<'a>, &'a [u8]), BagError> {
    let hlen = c.u32()? as usize;
    let header = HeaderMap::parse(c.take(hlen)?)?;
    let dlen = c.u32()? as usize;
    let data = c.take(dlen)?;
    Ok((header, data))
}

fn decompress(compression: &str, data: &[u8], size: usize) -> Result<Vec<u8>, BagError> {
    match compression {
        "none" => Ok(data.to_vec()),
        "bz2" => {
            let mut out = Vec::with_capacity(size);
            bzip2::read::BzDecoder::new(data)
                .read_to_end(&mut out)
                .map_err(|e| BagError::Decompress(format!("bz2: {e}")))?;
            Ok(out)
        }
        "lz4" => {
            let mut out = Vec::with_capacity(size);
            lz4::Decoder::new(data)
                .map_err(|e| BagError::Decompress(format!("lz4: {e}")))?
                .read_to_end(&mut out)
                .map_err(|e| BagError::Decompress(format!("lz4: {e}")))?;
            Ok(out)
        }
        other => Err(BagError::Decompress(format!(
            "unknown compression {other:?}"
        ))),
    }
}

// ---------------------------------------------------------------------------------------
// The reader
// ---------------------------------------------------------------------------------------

impl BagFile {
    /// Open a bag and parse its index (connections + chunk directory). No chunk is
    /// decompressed here.
    pub fn open<P: AsRef<Path>>(path: P) -> Result<BagFile, BagError> {
        let path = path.as_ref();
        let mut file = File::open(path).map_err(|source| BagError::Open {
            path: path.display().to_string(),
            source,
        })?;

        let mut magic = [0u8; MAGIC.len()];
        file.read_exact(&mut magic)?;
        if magic != *MAGIC {
            return Err(BagError::Format("not a ROS1 v2.0 bag (bad magic)"));
        }

        // Bag header record: tells us where the index section starts.
        let (header, _) = read_record_from_file(&mut file)?;
        let header = HeaderMap::parse(&header)?;
        if header.op()? != OP_BAG_HEADER {
            return Err(BagError::Format("first record is not the bag header"));
        }
        let index_pos = header.u64_field(b"index_pos")?;
        if index_pos == 0 {
            return Err(BagError::Format(
                "bag has no index (unfinished recording?); run `rosbag reindex`",
            ));
        }

        // The index section (connections + chunk infos) runs to EOF and is small;
        // read it whole.
        file.seek(SeekFrom::Start(index_pos))?;
        let mut index = Vec::new();
        file.read_to_end(&mut index)?;

        let mut connections = Vec::new();
        let mut chunks = Vec::new();
        let mut c = Cur::new(&index);
        while c.remaining() > 0 {
            let (header, data) = read_record(&mut c)?;
            match header.op()? {
                OP_CONNECTION => {
                    // Topic lives in the record header; message type in the embedded
                    // connection header (the data block).
                    let id = header.u32_field(b"conn")?;
                    let topic = header.str_field(b"topic")?.to_string();
                    let embedded = HeaderMap::parse(data)?;
                    let message_type = embedded.str_field(b"type")?.to_string();
                    connections.push(Connection {
                        id,
                        topic,
                        message_type,
                    });
                }
                OP_CHUNK_INFO => {
                    let pos = header.u64_field(b"chunk_pos")?;
                    let count = header.u32_field(b"count")? as usize;
                    let mut entries = Cur::new(data);
                    let mut conn_ids = Vec::with_capacity(count);
                    for _ in 0..count {
                        conn_ids.push(entries.u32()?);
                        entries.u32()?; // per-connection message count: unused
                    }
                    chunks.push(ChunkEntry {
                        pos,
                        connections: conn_ids,
                    });
                }
                _ => {}
            }
        }

        Ok(BagFile {
            file,
            connections,
            chunks,
        })
    }

    pub fn connections(&self) -> &[Connection] {
        &self.connections
    }

    /// Positions of the chunks containing at least one of `wanted` — the selection that
    /// makes extraction cost proportional to the *requested* data, not the bag size.
    fn select_chunks(&self, wanted: &BTreeSet<u32>) -> Vec<u64> {
        self.chunks
            .iter()
            .filter(|c| c.connections.iter().any(|id| wanted.contains(id)))
            .map(|c| c.pos)
            .collect()
    }

    /// Read + decompress the chunk record at `pos`, returning its payload.
    fn read_chunk_payload(&mut self, pos: u64) -> Result<Vec<u8>, BagError> {
        self.file.seek(SeekFrom::Start(pos))?;
        let (header, data) = read_record_from_file(&mut self.file)?;
        let header = HeaderMap::parse(&header)?;
        if header.op()? != OP_CHUNK {
            return Err(BagError::Format("chunk_pos does not point at a chunk"));
        }
        let compression = header.str_field(b"compression")?;
        let size = header.u32_field(b"size")? as usize;
        decompress(compression, &data, size)
    }

    /// Visit every message of the `wanted` connections, in file order, decompressing
    /// only the chunks that contain them.
    pub fn for_each_message(
        &mut self,
        wanted: &BTreeSet<u32>,
        mut visit: impl FnMut(u32, &[u8]) -> Result<(), BagError>,
    ) -> Result<(), BagError> {
        for pos in self.select_chunks(wanted) {
            let payload = self.read_chunk_payload(pos)?;
            visit_chunk_messages(&payload, wanted, &mut visit)?;
        }
        Ok(())
    }
}

/// Walk the records inside a decompressed chunk payload, dispatching message data.
fn visit_chunk_messages(
    payload: &[u8],
    wanted: &BTreeSet<u32>,
    visit: &mut impl FnMut(u32, &[u8]) -> Result<(), BagError>,
) -> Result<(), BagError> {
    let mut c = Cur::new(payload);
    while c.remaining() > 0 {
        let (header, data) = read_record(&mut c)?;
        if header.op()? == OP_MESSAGE_DATA {
            let conn = header.u32_field(b"conn")?;
            if wanted.contains(&conn) {
                visit(conn, data)?;
            }
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fixture() -> std::path::PathBuf {
        std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/mini.bag")
    }

    #[test]
    fn opens_and_indexes_the_fixture() {
        let bag = BagFile::open(fixture()).unwrap();
        let topics: Vec<(&str, &str)> = bag
            .connections()
            .iter()
            .map(|c| (c.topic.as_str(), c.message_type.as_str()))
            .collect();
        assert!(
            topics.contains(&("/d400/imu", "sensor_msgs/Imu")),
            "{topics:?}"
        );
        assert!(
            topics.contains(&("/scan", "sensor_msgs/LaserScan")),
            "{topics:?}"
        );
        assert!(!bag.chunks.is_empty());
    }

    #[test]
    fn chunk_selection_skips_unrelated_connections() {
        let bag = BagFile::open(fixture()).unwrap();
        let nonexistent: BTreeSet<u32> = [9999].into();
        assert!(bag.select_chunks(&nonexistent).is_empty());
    }

    #[test]
    fn visits_only_requested_messages() {
        let mut bag = BagFile::open(fixture()).unwrap();
        let imu_id = bag
            .connections()
            .iter()
            .find(|c| c.topic == "/d400/imu")
            .unwrap()
            .id;
        let mut count = 0;
        bag.for_each_message(&[imu_id].into(), |conn, _| {
            assert_eq!(conn, imu_id);
            count += 1;
            Ok(())
        })
        .unwrap();
        assert_eq!(count, 3);
    }

    #[test]
    fn rejects_non_bag_files(/* a text file is not a bag */) {
        let dir = std::env::temp_dir().join("slam-datasets-notabag");
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("notabag.txt");
        std::fs::write(
            &path,
            "definitely not a bag\nbut longer than the magic line",
        )
        .unwrap();
        let Err(err) = BagFile::open(&path) else {
            panic!("a text file must not open as a bag");
        };
        assert!(matches!(err, BagError::Format(_)));
    }
}
