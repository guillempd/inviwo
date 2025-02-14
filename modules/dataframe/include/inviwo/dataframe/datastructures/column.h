/*********************************************************************************
 *
 * Inviwo - Interactive Visualization Workshop
 *
 * Copyright (c) 2017-2021 Inviwo Foundation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************************/

#pragma once

#include <inviwo/dataframe/dataframemoduledefine.h>

#include <inviwo/core/datastructures/buffer/buffer.h>
#include <inviwo/core/datastructures/buffer/bufferram.h>
#include <inviwo/core/util/exception.h>
#include <inviwo/core/metadata/metadataowner.h>

#include <inviwo/dataframe/datastructures/datapoint.h>

#include <optional>
#include <iostream>

namespace inviwo {

class DataPointBase;
class BufferBase;

class IVW_MODULE_DATAFRAME_API InvalidConversion : public Exception {
public:
    InvalidConversion(const std::string& message = "",
                      ExceptionContext context = ExceptionContext())
        : Exception(message, context) {}
    virtual ~InvalidConversion() throw() {}
};

enum class ColumnType { Index, Ordinal, Categorical };

/**
 * @brief pure interface for representing a data column, i.e. a Buffer with a name
 */
class IVW_MODULE_DATAFRAME_API Column : public MetaDataOwner {
public:
    virtual ~Column() = default;

    virtual Column* clone() const = 0;

    virtual ColumnType getColumnType() const = 0;

    virtual const std::string& getHeader() const = 0;
    virtual void setHeader(std::string_view header) = 0;

    virtual void add(std::string_view value) = 0;
    /**
     * @brief appends all rows from column \p col
     * @param col
     */
    virtual void append(const Column& col) = 0;

    virtual std::shared_ptr<BufferBase> getBuffer() = 0;
    virtual std::shared_ptr<const BufferBase> getBuffer() const = 0;

    virtual size_t getSize() const = 0;

    virtual double getAsDouble(size_t idx) const = 0;
    virtual dvec2 getAsDVec2(size_t idx) const = 0;
    virtual dvec3 getAsDVec3(size_t idx) const = 0;
    virtual dvec4 getAsDVec4(size_t idx) const = 0;

    virtual std::string getAsString(size_t idx) const = 0;
    virtual std::shared_ptr<DataPointBase> get(size_t idx, bool getStringsAsStrings) const = 0;

    /**
     * Set a custom range for the column which can be used for normalization, plotting, color
     * mapping, etc.
     */
    virtual void setRange(dvec2 range);
    virtual void unsetRange();
    /**
     * Return the currently set column range. If no range has been set previously, the return value
     * will be std::nullopt.
     */
    std::optional<dvec2> getRange() const;

protected:
    Column() = default;
    std::optional<dvec2> columnRange_;
};

namespace columnutil {

/**
 * Return the column range for \p col. If the column's range is empty, the min/max values of the
 * underlying buffer are returned. If the buffer holds vectors, the componentwise minimum and
 * maximum are used.
 *
 * @return Column::getRange() if set, min/max values of getBuffer() otherwise
 */
IVW_MODULE_DATAFRAME_API dvec2 getRange(const Column& col);

}  // namespace columnutil

/**
 * @brief Data column used for plotting which represents a named buffer of type T. The name
 * is used as column header.
 */
template <typename T>
class TemplateColumn : public Column {
public:
    using type = T;

    TemplateColumn(std::string_view header,
                   std::shared_ptr<Buffer<T>> buffer = std::make_shared<Buffer<T>>());

    TemplateColumn(std::string_view header, std::vector<T> data);

    TemplateColumn(const TemplateColumn<T>& rhs);
    TemplateColumn(TemplateColumn<T>&& rhs);

    TemplateColumn<T>& operator=(const TemplateColumn<T>& rhs);
    TemplateColumn<T>& operator=(TemplateColumn<T>&& rhs);

    virtual TemplateColumn* clone() const override;

    virtual ~TemplateColumn() = default;

    virtual ColumnType getColumnType() const override;

    virtual const std::string& getHeader() const override;
    void setHeader(std::string_view header) override;

    virtual void add(const T& value);
    /**
     * \brief converts given value to type T, which is added to the column
     *
     * @param value
     * @throws InvalidConversion if the value cannot be converted to T
     */
    virtual void add(std::string_view value) override;

    /**
     * \copydoc Column::append(const Column&)
     * @throws Exception if data format does not match
     */
    virtual void append(const Column& col) override;

    virtual void set(size_t idx, const T& value);

    T get(size_t idx) const;
    T operator[](size_t idx) const;
    /**
     * \brief returns the data value for the given index.
     *
     * @param idx    index
     * @param getStringsAsStrings   if true, strings will be returned for categorical values
     *           instead of their internal representation. This will not affect other column types.
     *
     * \see CategoricalColumn
     */
    virtual std::shared_ptr<DataPointBase> get(size_t idx, bool getStringsAsStrings) const override;

    virtual double getAsDouble(size_t idx) const override;

    virtual dvec2 getAsDVec2(size_t idx) const override;

    virtual dvec3 getAsDVec3(size_t idx) const override;

    virtual dvec4 getAsDVec4(size_t idx) const override;

    void setBuffer(std::shared_ptr<Buffer<T>> buffer);

    virtual std::string getAsString(size_t idx) const override;

    virtual std::shared_ptr<BufferBase> getBuffer() override;
    virtual std::shared_ptr<const BufferBase> getBuffer() const override;

    std::shared_ptr<Buffer<T>> getTypedBuffer();
    std::shared_ptr<const Buffer<T>> getTypedBuffer() const;

    virtual size_t getSize() const override;

    auto begin() { return buffer_->getEditableRAMRepresentation()->getDataContainer().begin(); }
    auto end() { return buffer_->getEditableRAMRepresentation()->getDataContainer().end(); }
    auto begin() const { return buffer_->getRAMRepresentation()->getDataContainer().begin(); }
    auto end() const { return buffer_->getRAMRepresentation()->getDataContainer().end(); }

protected:
    std::string header_;
    std::shared_ptr<Buffer<T>> buffer_;
};

class IVW_MODULE_DATAFRAME_API IndexColumn : public TemplateColumn<std::uint32_t> {
public:
    IndexColumn(std::string_view header, std::shared_ptr<Buffer<std::uint32_t>> buffer =
                                             std::make_shared<Buffer<std::uint32_t>>());
    IndexColumn(std::string_view header, std::vector<std::uint32_t> data);
    IndexColumn(const IndexColumn& rhs) = default;
    IndexColumn(IndexColumn&& rhs) = default;

    IndexColumn& operator=(const IndexColumn& rhs) = default;
    IndexColumn& operator=(IndexColumn&& rhs) = default;

    virtual IndexColumn* clone() const override;

    virtual ~IndexColumn() = default;

    virtual ColumnType getColumnType() const override;
};

/**
 * \class CategoricalColumn
 * \brief Specialized data column representing categorical values, i.e. strings.
 * Categorical values are internally mapped to a number representation.
 *
 * For example:
 *    The data column "blue", "blue", "red", "yellow" might internally be represented
 *    by 0, 0, 1, 2.
 *    The original string values can be accessed using CategoricalColumn::get(index, true)
 *
 * \see TemplateColumn, \see CategoricalColumn::get()
 */
class IVW_MODULE_DATAFRAME_API CategoricalColumn : public TemplateColumn<std::uint32_t> {
public:
    CategoricalColumn(std::string_view header, const std::vector<std::string>& values = {});
    CategoricalColumn(const CategoricalColumn& rhs) = default;
    CategoricalColumn(CategoricalColumn&& rhs) = default;

    CategoricalColumn& operator=(const CategoricalColumn& rhs) = default;
    CategoricalColumn& operator=(CategoricalColumn&& rhs) = default;

    virtual CategoricalColumn* clone() const override;

    virtual ~CategoricalColumn() = default;

    virtual ColumnType getColumnType() const override;

    virtual std::string getAsString(size_t idx) const override;

    /**
     * \brief returns either the categorical value, i.e. a number representation, or
     * the actual string for the given index.
     *
     * @param idx    index
     * @param getStringsAsStrings   if true, a string will be returned instead of the
     *             internal representation. This will not affect other column types.
     *
     * \see TemplateColumn
     */
    virtual std::shared_ptr<DataPointBase> get(size_t idx, bool getStringsAsStrings) const override;

    using TemplateColumn<std::uint32_t>::set;
    virtual void set(size_t idx, const std::string& str);

    virtual void add(std::string_view value) override;

    /**
     * \brief \copybrief Column::append(const Column&) and builds a union of all
     * categorical values
     *
     * @param col
     * @throws Exception if data format does not match
     */
    virtual void append(const Column& col) override;

    /**
     * \brief append the categorical values given in \p data
     *
     * @param data    categorical values
     */
    void append(const std::vector<std::string>& data);

    /**
     * Returns the unique set of categorical values.
     */
    const std::vector<std::string>& getCategories() const { return lookUpTable_; }

    /**
     * \brief returns column contents as list of categorical values
     *
     * @return all categorical values stored in column
     */
    std::vector<std::string> getValues() const;

    /**
     * \brief add a category \p cat. It will not be added if the category already exists.
     *
     * @return index of the category
     */
    std::uint32_t addCategory(std::string_view cat);

private:
    virtual glm::uint32_t addOrGetID(std::string_view str);

    std::vector<std::string> lookUpTable_;
};

template <typename T>
TemplateColumn<T>::TemplateColumn(std::string_view header, std::shared_ptr<Buffer<T>> buffer)
    : header_(header), buffer_(buffer) {}

template <typename T>
TemplateColumn<T>::TemplateColumn(std::string_view header, std::vector<T> data)
    : header_(header), buffer_(util::makeBuffer(std::move(data))) {}

template <typename T>
TemplateColumn<T>::TemplateColumn(const TemplateColumn& rhs)
    : header_(rhs.getHeader())
    , buffer_(std::shared_ptr<Buffer<T>>(rhs.getTypedBuffer()->clone())) {}

template <typename T>
TemplateColumn<T>::TemplateColumn(TemplateColumn<T>&& rhs)
    : header_(std::move(rhs.header_)), buffer_(std::move(rhs.buffer_)) {}

template <typename T>
TemplateColumn<T>& TemplateColumn<T>::operator=(const TemplateColumn<T>& rhs) {
    if (this != &rhs) {
        header_ = rhs.getHeader();
        buffer_ = std::shared_ptr<Buffer<T>>(rhs.getTypedBuffer()->clone());
    }
    return *this;
}

template <typename T>
TemplateColumn<T>& TemplateColumn<T>::operator=(TemplateColumn<T>&& rhs) {
    if (this != &rhs) {
        header_ = std::move(rhs.header_);
        buffer_ = std::move(rhs.buffer_);
    }
    return *this;
}

template <typename T>
TemplateColumn<T>* TemplateColumn<T>::clone() const {
    return new TemplateColumn(*this);
}

template <typename T>
ColumnType TemplateColumn<T>::getColumnType() const {
    return ColumnType::Ordinal;
}

template <typename T>
const std::string& TemplateColumn<T>::getHeader() const {
    return header_;
}

template <typename T>
void TemplateColumn<T>::setHeader(std::string_view header) {
    header_ = header;
}

template <typename T>
void TemplateColumn<T>::add(const T& value) {
    buffer_->getEditableRAMRepresentation()->add(value);
}

namespace detail {

template <typename T, typename std::enable_if<std::is_integral<T>::value, int>::type = 0>
void add(Buffer<T>* buffer, std::string_view value) {
    T result;
    if (value.empty()) {
        result = T{0};  // no special value indicating missing data for integral types
    } else {
        std::stringstream stream;
        stream << value;
        stream >> result;
        if (stream.fail()) {
            throw InvalidConversion(fmt::format("cannot convert \"{}\" to target type", value));
        }
    }
    buffer->getEditableRAMRepresentation()->add(result);
}
// Specialization for float and double types, add NaN instead of throwing an error
template <typename T, typename std::enable_if<std::is_floating_point<T>::value, int>::type = 0>
void add(Buffer<T>* buffer, std::string_view value) {
    T result;
    std::stringstream stream;
    stream << value;
    stream >> result;
    if (stream.fail()) {
        buffer->getEditableRAMRepresentation()->add(std::numeric_limits<T>::quiet_NaN());
    } else {
        buffer->getEditableRAMRepresentation()->add(result);
    }
}

template <typename T,
          typename std::enable_if<!std::is_integral<T>::value && !std::is_floating_point<T>::value,
                                  int>::type = 0>
void add(Buffer<T>* /*buffer*/, std::string_view value) {
    throw InvalidConversion(
        fmt::format("conversion to target type not implemented (\"{}\")", value));
}

}  // namespace detail

template <typename T>
void TemplateColumn<T>::add(std::string_view value) {
    detail::add<T>(buffer_.get(), value);
}

template <typename T>
void TemplateColumn<T>::append(const Column& col) {
    if (auto srccol = dynamic_cast<const TemplateColumn<T>*>(&col)) {
        buffer_->getEditableRAMRepresentation()->append(
            srccol->buffer_->getRAMRepresentation()->getDataContainer());
    } else {
        throw Exception("data formats of columns do not match", IVW_CONTEXT);
    }
}

template <typename T>
void TemplateColumn<T>::set(size_t idx, const T& value) {
    buffer_->getEditableRAMRepresentation()->set(idx, value);
}

template <typename T>
T TemplateColumn<T>::get(size_t idx) const {
    auto val = buffer_->getRAMRepresentation()->getDataContainer()[idx];
    return val;
}

template <typename T>
double TemplateColumn<T>::getAsDouble(size_t idx) const {
    auto val = buffer_->getRAMRepresentation()->getDataContainer()[idx];
    return util::glm_convert<double>(val);
}

template <typename T>
dvec2 TemplateColumn<T>::getAsDVec2(size_t idx) const {
    auto val = buffer_->getRAMRepresentation()->getDataContainer()[idx];
    return util::glm_convert<dvec2>(val);
}

template <typename T>
dvec3 TemplateColumn<T>::getAsDVec3(size_t idx) const {
    auto val = buffer_->getRAMRepresentation()->getDataContainer()[idx];
    return util::glm_convert<dvec3>(val);
}

template <typename T>
dvec4 TemplateColumn<T>::getAsDVec4(size_t idx) const {
    auto val = buffer_->getRAMRepresentation()->getDataContainer()[idx];
    return util::glm_convert<dvec4>(val);
}

template <typename T>
void TemplateColumn<T>::setBuffer(std::shared_ptr<Buffer<T>> buffer) {
    buffer_ = buffer;
}

template <typename T>
std::string TemplateColumn<T>::getAsString(size_t idx) const {
    std::ostringstream ss;
    ss << buffer_->getRAMRepresentation()->get(idx);
    return ss.str();
}

template <typename T>
std::shared_ptr<DataPointBase> TemplateColumn<T>::get(size_t idx, bool) const {
    return std::make_shared<DataPoint<T>>(buffer_->getRAMRepresentation()->get(idx));
}

template <typename T>
T TemplateColumn<T>::operator[](const size_t idx) const {
    return get(idx);
}

template <typename T>
std::shared_ptr<BufferBase> TemplateColumn<T>::getBuffer() {
    return buffer_;
}

template <typename T>
std::shared_ptr<const BufferBase> TemplateColumn<T>::getBuffer() const {
    return buffer_;
}

template <typename T>
std::shared_ptr<Buffer<T>> TemplateColumn<T>::getTypedBuffer() {
    return buffer_;
}

template <typename T>
std::shared_ptr<const Buffer<T>> TemplateColumn<T>::getTypedBuffer() const {
    return buffer_;
}

template <typename T>
size_t TemplateColumn<T>::getSize() const {
    return buffer_->getSize();
}

}  // namespace inviwo
