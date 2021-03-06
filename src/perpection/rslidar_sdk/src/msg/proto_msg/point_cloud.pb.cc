// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: point_cloud.proto

#define INTERNAL_SUPPRESS_PROTOBUF_FIELD_DEPRECATION
#include "point_cloud.pb.h"

#include <algorithm>

#include <google/protobuf/stubs/common.h>
#include <google/protobuf/stubs/port.h>
#include <google/protobuf/stubs/once.h>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/wire_format_lite_inl.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/reflection_ops.h>
#include <google/protobuf/wire_format.h>
// @@protoc_insertion_point(includes)

namespace proto_msg {

namespace {

const ::google::protobuf::Descriptor* LidarPointCloud_descriptor_ = NULL;
const ::google::protobuf::internal::GeneratedMessageReflection*
  LidarPointCloud_reflection_ = NULL;

}  // namespace


void protobuf_AssignDesc_point_5fcloud_2eproto() GOOGLE_ATTRIBUTE_COLD;
void protobuf_AssignDesc_point_5fcloud_2eproto() {
  protobuf_AddDesc_point_5fcloud_2eproto();
  const ::google::protobuf::FileDescriptor* file =
    ::google::protobuf::DescriptorPool::generated_pool()->FindFileByName(
      "point_cloud.proto");
  GOOGLE_CHECK(file != NULL);
  LidarPointCloud_descriptor_ = file->message_type(0);
  static const int LidarPointCloud_offsets_[7] = {
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(LidarPointCloud, timestamp_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(LidarPointCloud, seq_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(LidarPointCloud, frame_id_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(LidarPointCloud, height_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(LidarPointCloud, width_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(LidarPointCloud, is_dense_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(LidarPointCloud, data_),
  };
  LidarPointCloud_reflection_ =
    ::google::protobuf::internal::GeneratedMessageReflection::NewGeneratedMessageReflection(
      LidarPointCloud_descriptor_,
      LidarPointCloud::default_instance_,
      LidarPointCloud_offsets_,
      GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(LidarPointCloud, _has_bits_[0]),
      -1,
      -1,
      sizeof(LidarPointCloud),
      GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(LidarPointCloud, _internal_metadata_),
      -1);
}

namespace {

GOOGLE_PROTOBUF_DECLARE_ONCE(protobuf_AssignDescriptors_once_);
inline void protobuf_AssignDescriptorsOnce() {
  ::google::protobuf::GoogleOnceInit(&protobuf_AssignDescriptors_once_,
                 &protobuf_AssignDesc_point_5fcloud_2eproto);
}

void protobuf_RegisterTypes(const ::std::string&) GOOGLE_ATTRIBUTE_COLD;
void protobuf_RegisterTypes(const ::std::string&) {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedMessage(
      LidarPointCloud_descriptor_, &LidarPointCloud::default_instance());
}

}  // namespace

void protobuf_ShutdownFile_point_5fcloud_2eproto() {
  delete LidarPointCloud::default_instance_;
  delete LidarPointCloud_reflection_;
}

void protobuf_AddDesc_point_5fcloud_2eproto() GOOGLE_ATTRIBUTE_COLD;
void protobuf_AddDesc_point_5fcloud_2eproto() {
  static bool already_here = false;
  if (already_here) return;
  already_here = true;
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  ::google::protobuf::DescriptorPool::InternalAddGeneratedFile(
    "\n\021point_cloud.proto\022\tproto_msg\"\202\001\n\017Lidar"
    "PointCloud\022\021\n\ttimestamp\030\001 \001(\001\022\013\n\003seq\030\002 \001"
    "(\r\022\020\n\010frame_id\030\003 \001(\t\022\016\n\006height\030\004 \001(\r\022\r\n\005"
    "width\030\005 \001(\r\022\020\n\010is_dense\030\006 \001(\010\022\014\n\004data\030\007 "
    "\003(\002", 163);
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedFile(
    "point_cloud.proto", &protobuf_RegisterTypes);
  LidarPointCloud::default_instance_ = new LidarPointCloud();
  LidarPointCloud::default_instance_->InitAsDefaultInstance();
  ::google::protobuf::internal::OnShutdown(&protobuf_ShutdownFile_point_5fcloud_2eproto);
}

// Force AddDescriptors() to be called at static initialization time.
struct StaticDescriptorInitializer_point_5fcloud_2eproto {
  StaticDescriptorInitializer_point_5fcloud_2eproto() {
    protobuf_AddDesc_point_5fcloud_2eproto();
  }
} static_descriptor_initializer_point_5fcloud_2eproto_;

// ===================================================================

#if !defined(_MSC_VER) || _MSC_VER >= 1900
const int LidarPointCloud::kTimestampFieldNumber;
const int LidarPointCloud::kSeqFieldNumber;
const int LidarPointCloud::kFrameIdFieldNumber;
const int LidarPointCloud::kHeightFieldNumber;
const int LidarPointCloud::kWidthFieldNumber;
const int LidarPointCloud::kIsDenseFieldNumber;
const int LidarPointCloud::kDataFieldNumber;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

LidarPointCloud::LidarPointCloud()
  : ::google::protobuf::Message(), _internal_metadata_(NULL) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:proto_msg.LidarPointCloud)
}

void LidarPointCloud::InitAsDefaultInstance() {
}

LidarPointCloud::LidarPointCloud(const LidarPointCloud& from)
  : ::google::protobuf::Message(),
    _internal_metadata_(NULL) {
  SharedCtor();
  MergeFrom(from);
  // @@protoc_insertion_point(copy_constructor:proto_msg.LidarPointCloud)
}

void LidarPointCloud::SharedCtor() {
  ::google::protobuf::internal::GetEmptyString();
  _cached_size_ = 0;
  timestamp_ = 0;
  seq_ = 0u;
  frame_id_.UnsafeSetDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  height_ = 0u;
  width_ = 0u;
  is_dense_ = false;
  ::memset(_has_bits_, 0, sizeof(_has_bits_));
}

LidarPointCloud::~LidarPointCloud() {
  // @@protoc_insertion_point(destructor:proto_msg.LidarPointCloud)
  SharedDtor();
}

void LidarPointCloud::SharedDtor() {
  frame_id_.DestroyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  if (this != default_instance_) {
  }
}

void LidarPointCloud::SetCachedSize(int size) const {
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
}
const ::google::protobuf::Descriptor* LidarPointCloud::descriptor() {
  protobuf_AssignDescriptorsOnce();
  return LidarPointCloud_descriptor_;
}

const LidarPointCloud& LidarPointCloud::default_instance() {
  if (default_instance_ == NULL) protobuf_AddDesc_point_5fcloud_2eproto();
  return *default_instance_;
}

LidarPointCloud* LidarPointCloud::default_instance_ = NULL;

LidarPointCloud* LidarPointCloud::New(::google::protobuf::Arena* arena) const {
  LidarPointCloud* n = new LidarPointCloud;
  if (arena != NULL) {
    arena->Own(n);
  }
  return n;
}

void LidarPointCloud::Clear() {
// @@protoc_insertion_point(message_clear_start:proto_msg.LidarPointCloud)
#if defined(__clang__)
#define ZR_HELPER_(f) \
  _Pragma("clang diagnostic push") \
  _Pragma("clang diagnostic ignored \"-Winvalid-offsetof\"") \
  __builtin_offsetof(LidarPointCloud, f) \
  _Pragma("clang diagnostic pop")
#else
#define ZR_HELPER_(f) reinterpret_cast<char*>(\
  &reinterpret_cast<LidarPointCloud*>(16)->f)
#endif

#define ZR_(first, last) do {\
  ::memset(&first, 0,\
           ZR_HELPER_(last) - ZR_HELPER_(first) + sizeof(last));\
} while (0)

  if (_has_bits_[0 / 32] & 63u) {
    ZR_(seq_, is_dense_);
    timestamp_ = 0;
    if (has_frame_id()) {
      frame_id_.ClearToEmptyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
    }
  }

#undef ZR_HELPER_
#undef ZR_

  data_.Clear();
  ::memset(_has_bits_, 0, sizeof(_has_bits_));
  if (_internal_metadata_.have_unknown_fields()) {
    mutable_unknown_fields()->Clear();
  }
}

bool LidarPointCloud::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!GOOGLE_PREDICT_TRUE(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:proto_msg.LidarPointCloud)
  for (;;) {
    ::std::pair< ::google::protobuf::uint32, bool> p = input->ReadTagWithCutoff(127);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // optional double timestamp = 1;
      case 1: {
        if (tag == 9) {
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &timestamp_)));
          set_has_timestamp();
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(16)) goto parse_seq;
        break;
      }

      // optional uint32 seq = 2;
      case 2: {
        if (tag == 16) {
         parse_seq:
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   ::google::protobuf::uint32, ::google::protobuf::internal::WireFormatLite::TYPE_UINT32>(
                 input, &seq_)));
          set_has_seq();
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(26)) goto parse_frame_id;
        break;
      }

      // optional string frame_id = 3;
      case 3: {
        if (tag == 26) {
         parse_frame_id:
          DO_(::google::protobuf::internal::WireFormatLite::ReadString(
                input, this->mutable_frame_id()));
          ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
            this->frame_id().data(), this->frame_id().length(),
            ::google::protobuf::internal::WireFormat::PARSE,
            "proto_msg.LidarPointCloud.frame_id");
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(32)) goto parse_height;
        break;
      }

      // optional uint32 height = 4;
      case 4: {
        if (tag == 32) {
         parse_height:
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   ::google::protobuf::uint32, ::google::protobuf::internal::WireFormatLite::TYPE_UINT32>(
                 input, &height_)));
          set_has_height();
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(40)) goto parse_width;
        break;
      }

      // optional uint32 width = 5;
      case 5: {
        if (tag == 40) {
         parse_width:
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   ::google::protobuf::uint32, ::google::protobuf::internal::WireFormatLite::TYPE_UINT32>(
                 input, &width_)));
          set_has_width();
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(48)) goto parse_is_dense;
        break;
      }

      // optional bool is_dense = 6;
      case 6: {
        if (tag == 48) {
         parse_is_dense:
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   bool, ::google::protobuf::internal::WireFormatLite::TYPE_BOOL>(
                 input, &is_dense_)));
          set_has_is_dense();
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(61)) goto parse_data;
        break;
      }

      // repeated float data = 7;
      case 7: {
        if (tag == 61) {
         parse_data:
          DO_((::google::protobuf::internal::WireFormatLite::ReadRepeatedPrimitive<
                   float, ::google::protobuf::internal::WireFormatLite::TYPE_FLOAT>(
                 1, 61, input, this->mutable_data())));
        } else if (tag == 58) {
          DO_((::google::protobuf::internal::WireFormatLite::ReadPackedPrimitiveNoInline<
                   float, ::google::protobuf::internal::WireFormatLite::TYPE_FLOAT>(
                 input, this->mutable_data())));
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(61)) goto parse_data;
        if (input->ExpectAtEnd()) goto success;
        break;
      }

      default: {
      handle_unusual:
        if (tag == 0 ||
            ::google::protobuf::internal::WireFormatLite::GetTagWireType(tag) ==
            ::google::protobuf::internal::WireFormatLite::WIRETYPE_END_GROUP) {
          goto success;
        }
        DO_(::google::protobuf::internal::WireFormat::SkipField(
              input, tag, mutable_unknown_fields()));
        break;
      }
    }
  }
success:
  // @@protoc_insertion_point(parse_success:proto_msg.LidarPointCloud)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:proto_msg.LidarPointCloud)
  return false;
#undef DO_
}

void LidarPointCloud::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:proto_msg.LidarPointCloud)
  // optional double timestamp = 1;
  if (has_timestamp()) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(1, this->timestamp(), output);
  }

  // optional uint32 seq = 2;
  if (has_seq()) {
    ::google::protobuf::internal::WireFormatLite::WriteUInt32(2, this->seq(), output);
  }

  // optional string frame_id = 3;
  if (has_frame_id()) {
    ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
      this->frame_id().data(), this->frame_id().length(),
      ::google::protobuf::internal::WireFormat::SERIALIZE,
      "proto_msg.LidarPointCloud.frame_id");
    ::google::protobuf::internal::WireFormatLite::WriteStringMaybeAliased(
      3, this->frame_id(), output);
  }

  // optional uint32 height = 4;
  if (has_height()) {
    ::google::protobuf::internal::WireFormatLite::WriteUInt32(4, this->height(), output);
  }

  // optional uint32 width = 5;
  if (has_width()) {
    ::google::protobuf::internal::WireFormatLite::WriteUInt32(5, this->width(), output);
  }

  // optional bool is_dense = 6;
  if (has_is_dense()) {
    ::google::protobuf::internal::WireFormatLite::WriteBool(6, this->is_dense(), output);
  }

  // repeated float data = 7;
  for (int i = 0; i < this->data_size(); i++) {
    ::google::protobuf::internal::WireFormatLite::WriteFloat(
      7, this->data(i), output);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        unknown_fields(), output);
  }
  // @@protoc_insertion_point(serialize_end:proto_msg.LidarPointCloud)
}

::google::protobuf::uint8* LidarPointCloud::InternalSerializeWithCachedSizesToArray(
    bool deterministic, ::google::protobuf::uint8* target) const {
  // @@protoc_insertion_point(serialize_to_array_start:proto_msg.LidarPointCloud)
  // optional double timestamp = 1;
  if (has_timestamp()) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(1, this->timestamp(), target);
  }

  // optional uint32 seq = 2;
  if (has_seq()) {
    target = ::google::protobuf::internal::WireFormatLite::WriteUInt32ToArray(2, this->seq(), target);
  }

  // optional string frame_id = 3;
  if (has_frame_id()) {
    ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
      this->frame_id().data(), this->frame_id().length(),
      ::google::protobuf::internal::WireFormat::SERIALIZE,
      "proto_msg.LidarPointCloud.frame_id");
    target =
      ::google::protobuf::internal::WireFormatLite::WriteStringToArray(
        3, this->frame_id(), target);
  }

  // optional uint32 height = 4;
  if (has_height()) {
    target = ::google::protobuf::internal::WireFormatLite::WriteUInt32ToArray(4, this->height(), target);
  }

  // optional uint32 width = 5;
  if (has_width()) {
    target = ::google::protobuf::internal::WireFormatLite::WriteUInt32ToArray(5, this->width(), target);
  }

  // optional bool is_dense = 6;
  if (has_is_dense()) {
    target = ::google::protobuf::internal::WireFormatLite::WriteBoolToArray(6, this->is_dense(), target);
  }

  // repeated float data = 7;
  for (int i = 0; i < this->data_size(); i++) {
    target = ::google::protobuf::internal::WireFormatLite::
      WriteFloatToArray(7, this->data(i), target);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        unknown_fields(), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:proto_msg.LidarPointCloud)
  return target;
}

int LidarPointCloud::ByteSize() const {
// @@protoc_insertion_point(message_byte_size_start:proto_msg.LidarPointCloud)
  int total_size = 0;

  if (_has_bits_[0 / 32] & 63u) {
    // optional double timestamp = 1;
    if (has_timestamp()) {
      total_size += 1 + 8;
    }

    // optional uint32 seq = 2;
    if (has_seq()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::UInt32Size(
          this->seq());
    }

    // optional string frame_id = 3;
    if (has_frame_id()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::StringSize(
          this->frame_id());
    }

    // optional uint32 height = 4;
    if (has_height()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::UInt32Size(
          this->height());
    }

    // optional uint32 width = 5;
    if (has_width()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::UInt32Size(
          this->width());
    }

    // optional bool is_dense = 6;
    if (has_is_dense()) {
      total_size += 1 + 1;
    }

  }
  // repeated float data = 7;
  {
    int data_size = 0;
    data_size = 4 * this->data_size();
    total_size += 1 * this->data_size() + data_size;
  }

  if (_internal_metadata_.have_unknown_fields()) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        unknown_fields());
  }
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = total_size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
  return total_size;
}

void LidarPointCloud::MergeFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:proto_msg.LidarPointCloud)
  if (GOOGLE_PREDICT_FALSE(&from == this)) {
    ::google::protobuf::internal::MergeFromFail(__FILE__, __LINE__);
  }
  const LidarPointCloud* source = 
      ::google::protobuf::internal::DynamicCastToGenerated<const LidarPointCloud>(
          &from);
  if (source == NULL) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:proto_msg.LidarPointCloud)
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:proto_msg.LidarPointCloud)
    MergeFrom(*source);
  }
}

void LidarPointCloud::MergeFrom(const LidarPointCloud& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:proto_msg.LidarPointCloud)
  if (GOOGLE_PREDICT_FALSE(&from == this)) {
    ::google::protobuf::internal::MergeFromFail(__FILE__, __LINE__);
  }
  data_.MergeFrom(from.data_);
  if (from._has_bits_[0 / 32] & (0xffu << (0 % 32))) {
    if (from.has_timestamp()) {
      set_timestamp(from.timestamp());
    }
    if (from.has_seq()) {
      set_seq(from.seq());
    }
    if (from.has_frame_id()) {
      set_has_frame_id();
      frame_id_.AssignWithDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), from.frame_id_);
    }
    if (from.has_height()) {
      set_height(from.height());
    }
    if (from.has_width()) {
      set_width(from.width());
    }
    if (from.has_is_dense()) {
      set_is_dense(from.is_dense());
    }
  }
  if (from._internal_metadata_.have_unknown_fields()) {
    mutable_unknown_fields()->MergeFrom(from.unknown_fields());
  }
}

void LidarPointCloud::CopyFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:proto_msg.LidarPointCloud)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void LidarPointCloud::CopyFrom(const LidarPointCloud& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:proto_msg.LidarPointCloud)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool LidarPointCloud::IsInitialized() const {

  return true;
}

void LidarPointCloud::Swap(LidarPointCloud* other) {
  if (other == this) return;
  InternalSwap(other);
}
void LidarPointCloud::InternalSwap(LidarPointCloud* other) {
  std::swap(timestamp_, other->timestamp_);
  std::swap(seq_, other->seq_);
  frame_id_.Swap(&other->frame_id_);
  std::swap(height_, other->height_);
  std::swap(width_, other->width_);
  std::swap(is_dense_, other->is_dense_);
  data_.UnsafeArenaSwap(&other->data_);
  std::swap(_has_bits_[0], other->_has_bits_[0]);
  _internal_metadata_.Swap(&other->_internal_metadata_);
  std::swap(_cached_size_, other->_cached_size_);
}

::google::protobuf::Metadata LidarPointCloud::GetMetadata() const {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::Metadata metadata;
  metadata.descriptor = LidarPointCloud_descriptor_;
  metadata.reflection = LidarPointCloud_reflection_;
  return metadata;
}

#if PROTOBUF_INLINE_NOT_IN_HEADERS
// LidarPointCloud

// optional double timestamp = 1;
bool LidarPointCloud::has_timestamp() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
void LidarPointCloud::set_has_timestamp() {
  _has_bits_[0] |= 0x00000001u;
}
void LidarPointCloud::clear_has_timestamp() {
  _has_bits_[0] &= ~0x00000001u;
}
void LidarPointCloud::clear_timestamp() {
  timestamp_ = 0;
  clear_has_timestamp();
}
 double LidarPointCloud::timestamp() const {
  // @@protoc_insertion_point(field_get:proto_msg.LidarPointCloud.timestamp)
  return timestamp_;
}
 void LidarPointCloud::set_timestamp(double value) {
  set_has_timestamp();
  timestamp_ = value;
  // @@protoc_insertion_point(field_set:proto_msg.LidarPointCloud.timestamp)
}

// optional uint32 seq = 2;
bool LidarPointCloud::has_seq() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
void LidarPointCloud::set_has_seq() {
  _has_bits_[0] |= 0x00000002u;
}
void LidarPointCloud::clear_has_seq() {
  _has_bits_[0] &= ~0x00000002u;
}
void LidarPointCloud::clear_seq() {
  seq_ = 0u;
  clear_has_seq();
}
 ::google::protobuf::uint32 LidarPointCloud::seq() const {
  // @@protoc_insertion_point(field_get:proto_msg.LidarPointCloud.seq)
  return seq_;
}
 void LidarPointCloud::set_seq(::google::protobuf::uint32 value) {
  set_has_seq();
  seq_ = value;
  // @@protoc_insertion_point(field_set:proto_msg.LidarPointCloud.seq)
}

// optional string frame_id = 3;
bool LidarPointCloud::has_frame_id() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
void LidarPointCloud::set_has_frame_id() {
  _has_bits_[0] |= 0x00000004u;
}
void LidarPointCloud::clear_has_frame_id() {
  _has_bits_[0] &= ~0x00000004u;
}
void LidarPointCloud::clear_frame_id() {
  frame_id_.ClearToEmptyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  clear_has_frame_id();
}
 const ::std::string& LidarPointCloud::frame_id() const {
  // @@protoc_insertion_point(field_get:proto_msg.LidarPointCloud.frame_id)
  return frame_id_.GetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
 void LidarPointCloud::set_frame_id(const ::std::string& value) {
  set_has_frame_id();
  frame_id_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), value);
  // @@protoc_insertion_point(field_set:proto_msg.LidarPointCloud.frame_id)
}
 void LidarPointCloud::set_frame_id(const char* value) {
  set_has_frame_id();
  frame_id_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::string(value));
  // @@protoc_insertion_point(field_set_char:proto_msg.LidarPointCloud.frame_id)
}
 void LidarPointCloud::set_frame_id(const char* value, size_t size) {
  set_has_frame_id();
  frame_id_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(),
      ::std::string(reinterpret_cast<const char*>(value), size));
  // @@protoc_insertion_point(field_set_pointer:proto_msg.LidarPointCloud.frame_id)
}
 ::std::string* LidarPointCloud::mutable_frame_id() {
  set_has_frame_id();
  // @@protoc_insertion_point(field_mutable:proto_msg.LidarPointCloud.frame_id)
  return frame_id_.MutableNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
 ::std::string* LidarPointCloud::release_frame_id() {
  // @@protoc_insertion_point(field_release:proto_msg.LidarPointCloud.frame_id)
  clear_has_frame_id();
  return frame_id_.ReleaseNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
 void LidarPointCloud::set_allocated_frame_id(::std::string* frame_id) {
  if (frame_id != NULL) {
    set_has_frame_id();
  } else {
    clear_has_frame_id();
  }
  frame_id_.SetAllocatedNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), frame_id);
  // @@protoc_insertion_point(field_set_allocated:proto_msg.LidarPointCloud.frame_id)
}

// optional uint32 height = 4;
bool LidarPointCloud::has_height() const {
  return (_has_bits_[0] & 0x00000008u) != 0;
}
void LidarPointCloud::set_has_height() {
  _has_bits_[0] |= 0x00000008u;
}
void LidarPointCloud::clear_has_height() {
  _has_bits_[0] &= ~0x00000008u;
}
void LidarPointCloud::clear_height() {
  height_ = 0u;
  clear_has_height();
}
 ::google::protobuf::uint32 LidarPointCloud::height() const {
  // @@protoc_insertion_point(field_get:proto_msg.LidarPointCloud.height)
  return height_;
}
 void LidarPointCloud::set_height(::google::protobuf::uint32 value) {
  set_has_height();
  height_ = value;
  // @@protoc_insertion_point(field_set:proto_msg.LidarPointCloud.height)
}

// optional uint32 width = 5;
bool LidarPointCloud::has_width() const {
  return (_has_bits_[0] & 0x00000010u) != 0;
}
void LidarPointCloud::set_has_width() {
  _has_bits_[0] |= 0x00000010u;
}
void LidarPointCloud::clear_has_width() {
  _has_bits_[0] &= ~0x00000010u;
}
void LidarPointCloud::clear_width() {
  width_ = 0u;
  clear_has_width();
}
 ::google::protobuf::uint32 LidarPointCloud::width() const {
  // @@protoc_insertion_point(field_get:proto_msg.LidarPointCloud.width)
  return width_;
}
 void LidarPointCloud::set_width(::google::protobuf::uint32 value) {
  set_has_width();
  width_ = value;
  // @@protoc_insertion_point(field_set:proto_msg.LidarPointCloud.width)
}

// optional bool is_dense = 6;
bool LidarPointCloud::has_is_dense() const {
  return (_has_bits_[0] & 0x00000020u) != 0;
}
void LidarPointCloud::set_has_is_dense() {
  _has_bits_[0] |= 0x00000020u;
}
void LidarPointCloud::clear_has_is_dense() {
  _has_bits_[0] &= ~0x00000020u;
}
void LidarPointCloud::clear_is_dense() {
  is_dense_ = false;
  clear_has_is_dense();
}
 bool LidarPointCloud::is_dense() const {
  // @@protoc_insertion_point(field_get:proto_msg.LidarPointCloud.is_dense)
  return is_dense_;
}
 void LidarPointCloud::set_is_dense(bool value) {
  set_has_is_dense();
  is_dense_ = value;
  // @@protoc_insertion_point(field_set:proto_msg.LidarPointCloud.is_dense)
}

// repeated float data = 7;
int LidarPointCloud::data_size() const {
  return data_.size();
}
void LidarPointCloud::clear_data() {
  data_.Clear();
}
 float LidarPointCloud::data(int index) const {
  // @@protoc_insertion_point(field_get:proto_msg.LidarPointCloud.data)
  return data_.Get(index);
}
 void LidarPointCloud::set_data(int index, float value) {
  data_.Set(index, value);
  // @@protoc_insertion_point(field_set:proto_msg.LidarPointCloud.data)
}
 void LidarPointCloud::add_data(float value) {
  data_.Add(value);
  // @@protoc_insertion_point(field_add:proto_msg.LidarPointCloud.data)
}
 const ::google::protobuf::RepeatedField< float >&
LidarPointCloud::data() const {
  // @@protoc_insertion_point(field_list:proto_msg.LidarPointCloud.data)
  return data_;
}
 ::google::protobuf::RepeatedField< float >*
LidarPointCloud::mutable_data() {
  // @@protoc_insertion_point(field_mutable_list:proto_msg.LidarPointCloud.data)
  return &data_;
}

#endif  // PROTOBUF_INLINE_NOT_IN_HEADERS

// @@protoc_insertion_point(namespace_scope)

}  // namespace proto_msg

// @@protoc_insertion_point(global_scope)
