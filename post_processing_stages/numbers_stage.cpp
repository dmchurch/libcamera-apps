#include <libcamera/stream.h>
#include <bitset>

#include "core/libcamera_app.hpp"

#include "post_processing_stages/post_processing_stage.hpp"

#define BITS 32

/////////////////////////////////////////////////////
///////////////// COORDINATE SYSTEM /////////////////
/////////////////////////////////////////////////////
namespace {
	struct BitPos {
		short offset; // Positive direction is DOWN, as with iterating through an array of pixel-rows
		short shift;  // Positive direction is LEFT, as with standard binary-number representation

		constexpr BitPos(): offset(SHRT_MIN), shift(SHRT_MIN) { }
		constexpr BitPos(int offset, int shift = 0): offset(offset), shift(shift) { }

		constexpr operator bool() const { return offset != SHRT_MIN && shift != SHRT_MIN; }
		constexpr bool operator==(BitPos rhs) const { return offset == rhs.offset && shift == rhs.shift; }
		constexpr bool operator!=(BitPos rhs) const { return offset != rhs.offset || shift != rhs.shift; }

		// single-axis comparisons
		constexpr bool below(BitPos rhs) const { return offset > rhs.offset; }
		constexpr bool above(BitPos rhs) const { return offset < rhs.offset; }
		constexpr bool left_of(BitPos rhs) const { return shift > rhs.shift; }
		constexpr bool right_of(BitPos rhs) const { return shift < rhs.shift; }
		constexpr bool at_or_below(BitPos rhs) const { return offset >= rhs.offset; }
		constexpr bool at_or_above(BitPos rhs) const { return offset <= rhs.offset; }
		constexpr bool at_or_left_of(BitPos rhs) const { return shift >= rhs.shift; }
		constexpr bool at_or_right_of(BitPos rhs) const { return shift <= rhs.shift; }

		// partial ordering: BOTH coordinates must satisfy the inequality
		constexpr bool operator<(BitPos rhs) const { return offset < rhs.offset && shift < rhs.shift; }
		constexpr bool operator<=(BitPos rhs) const { return offset <= rhs.offset && shift <= rhs.shift; }
		constexpr bool operator>(BitPos rhs) const { return offset > rhs.offset && shift > rhs.shift; }
		constexpr bool operator>=(BitPos rhs) const { return offset >= rhs.offset && shift >= rhs.shift; }

		// operators +, -, <<, >> on int: shift position down, up, left, or right
		constexpr BitPos &operator+=(int o) { offset += o; return *this; }
		constexpr BitPos &operator-=(int o) { offset -= o; return *this; }
		constexpr BitPos &operator<<=(int s) { shift += s; return *this; }
		constexpr BitPos &operator>>=(int s) { shift -= s; return *this; }
		friend constexpr BitPos operator+(BitPos bp, int o) { return BitPos(bp) += o; }
		friend constexpr BitPos operator-(BitPos bp, int o) { return BitPos(bp) -= o; }
		friend constexpr BitPos operator<<(BitPos bp, int o) { return BitPos(bp) <<= o; }
		friend constexpr BitPos operator>>(BitPos bp, int o) { return BitPos(bp) >>= o; }

		// operators +, - on BitPos: shift along both axes
		constexpr BitPos &operator+=(BitPos rhs) { return (*this += rhs.offset) <<= rhs.shift; }
		constexpr BitPos &operator-=(BitPos rhs) { return (*this -= rhs.offset) >>= rhs.shift; }
		friend constexpr BitPos operator+(BitPos lhs, BitPos rhs) { return BitPos(lhs) += rhs; }
		friend constexpr BitPos operator-(BitPos lhs, BitPos rhs) { return BitPos(lhs) -= rhs; }
	};
	static_assert(sizeof(BitPos) == 4, "BitPos doesn't fit in 32 bits");

	struct BitRange {
		BitPos origin; // actually the upper-right corner
		short height;
		short width;

		constexpr BitRange(): height(0), width(0) { }
		constexpr explicit BitRange(int height, int width = BITS, int offset = 0, int shift = 0): origin(offset, shift), height(height), width(width) { }
		constexpr BitRange(int height, int width, BitPos origin): origin(origin), height(height), width(width) { }

		constexpr int offset_min() const { return origin.offset; }
		constexpr int offset_max() const { return origin.offset + height - 1; }
		constexpr int shift_min() const { return origin.shift; }
		constexpr int shift_max() const { return origin.shift + width - 1; }
		constexpr BitPos pos_min() const { return origin; }
		constexpr BitPos pos_max() const { return {offset_max(), shift_max()}; }

		constexpr operator bool() const { return origin && height > 0 && width > 0; }

		constexpr bool contains(BitPos pos) { return pos >= pos_min() && pos <= pos_max(); }
		constexpr bool contains(BitRange other) { return other.pos_min() >= pos_min() && other.pos_max() <= pos_max(); }
		constexpr bool intersects(BitRange other) { return bool(*this * other); }

		constexpr BitRange &center() { origin.shift = (BITS - width) / 2; return *this; }
		constexpr BitRange &to_origin() { origin = {0, 0}; return *this; }
		constexpr BitRange centered() const { return BitRange(*this).center(); }
		constexpr BitRange at_origin() const { return BitRange(*this).to_origin(); }

		// operators +, -, <<, >> on int: shift range (origin) down, up, left, or right
		constexpr BitRange &operator+=(int o) { origin += o; return *this; }
		constexpr BitRange &operator-=(int o) { origin -= o; return *this; }
		constexpr BitRange &operator<<=(int s) { origin <<= s; return *this; }
		constexpr BitRange &operator>>=(int s) { origin <<= s; return *this; }
		friend constexpr BitRange operator+(BitRange br, int o) { return BitRange(br) += o; }
		friend constexpr BitRange operator-(BitRange br, int o) { return BitRange(br) -= o; }
		friend constexpr BitRange operator<<(BitRange br, int o) { return BitRange(br) <<= o; }
		friend constexpr BitRange operator>>(BitRange br, int o) { return BitRange(br) >>= o; }

		// operators +, - on BitPos: shift along both axes
		constexpr BitRange &operator+=(BitPos rhs) { origin += rhs; return *this; }
		constexpr BitRange &operator-=(BitPos rhs) { origin -= rhs; return *this; }
		friend constexpr BitRange operator+(BitRange lhs, BitPos rhs) { return BitRange(lhs) += rhs; }
		friend constexpr BitRange operator-(BitRange lhs, BitPos rhs) { return BitRange(lhs) -= rhs; }

		// operators +, *, -, / on BitRange: contiguous union, intersection, asymmetric convex difference by offset, asymmetric convex difference by shift
		constexpr BitRange &operator+=(BitRange rhs) {
			int omin = std::min(offset_min(), rhs.offset_min());
			int omax = std::max(offset_max(), rhs.offset_max());
			int smin = std::min(shift_min(), rhs.shift_min());
			int smax = std::max(shift_max(), rhs.shift_max());
			origin = {omin, smin};
			height = omax - omin + 1;
			width = smax - smin + 1;
			return *this;
		}
		constexpr BitRange &operator*=(BitRange rhs) {
			int omin = std::max(offset_min(), rhs.offset_min());
			int omax = std::min(offset_max(), rhs.offset_max());
			int smin = std::max(shift_min(), rhs.shift_min());
			int smax = std::min(shift_max(), rhs.shift_max());
			origin = {omin, smin};
			height = omax - omin + 1;
			width = smax - smin + 1;
			return *this;
		}
		constexpr BitRange &operator-=(BitRange rhs) {
			if (rhs.origin.at_or_below(origin)) {
				// keep our top segment
				height = std::min(int(height), rhs.origin.offset - origin.offset);
			} else {
				// keep our bottom segment
				int omin = std::max(offset_min(), rhs.offset_max() + 1);
				int omax = offset_max();
				origin.offset = omin;
				height = omax - omin + 1;
			}
			return *this;
		}
		constexpr BitRange &operator/=(BitRange rhs) {
			if (rhs.origin.at_or_left_of(origin)) {
				// keep our right segment
				width = std::min(int(width), rhs.origin.shift - origin.shift);
			} else {
				// keep our left segment
				int smin = std::max(shift_min(), rhs.shift_max() + 1);
				int smax = shift_max();
				origin.shift = smin;
				width = smax - smin + 1;
			}
			return *this;
		}
		friend constexpr BitRange operator+(BitRange lhs, BitRange rhs) { return BitRange(lhs) += rhs; };
		friend constexpr BitRange operator*(BitRange lhs, BitRange rhs) { return BitRange(lhs) *= rhs; };
		friend constexpr BitRange operator-(BitRange lhs, BitRange rhs) { return BitRange(lhs) -= rhs; };
		friend constexpr BitRange operator/(BitRange lhs, BitRange rhs) { return BitRange(lhs) /= rhs; };
	};
	static_assert(sizeof(BitRange) == 8, "BitRange doesn't fit in 64 bits");
}

/////////////////////////////////////////////////////
//////////////// DATA REPRESENTATION ////////////////
/////////////////////////////////////////////////////
namespace {
	class BitRow: public std::bitset<BITS> {
		using bitset::bitset;
	public:
		constexpr BitRow(bitset bs) noexcept: bitset(bs) {}
		BitRow operator<<(int shift) const { return bitset::operator<<(shift); }
		BitRow operator>>(int shift) const { return bitset::operator>>(shift); }
		friend std::ostream &operator<<(std::ostream &os, BitRow br) { return os << br.to_string('.','#'); }
	};

	class BitMatrix {
	public:
		BitMatrix(): height_(0), width_(BITS) {}
		BitMatrix(int height, int width = BITS): height_(height), width_(width), data_(std::make_unique<BitRow[]>(height)) {}
		constexpr short height() const { return height_; }
		constexpr short width() const { return width_; }
		const BitRow operator[](int i) const { return data_[i]; }
		BitRow &operator[](int i) { return data_[i]; }
		bool operator[](BitPos p) const { return (*this)[p.offset][p.shift]; }
		BitRow *begin() & { return &data_[0]; }
		const BitRow *begin() const & { return &data_[0]; }
		const BitRow *cbegin() const & { return &data_[0]; }
		BitRow *end() & { return &data_[height_]; }
		const BitRow *end() const & { return &data_[height_]; }
		const BitRow *cend() const & { return &data_[height_]; }

		void set() { for (int i = 0; i < height_; i++) data_[i].set(); }
		void set(int offset, int shift, bool value = true) { data_[offset].set(shift, value); }
		void set(BitPos pos, bool value = true) { set(pos.offset, pos.shift, value); }
		void reset() { for (int i = 0; i < height_; i++) data_[i].reset(); }
		void reset(int offset, int shift) { data_[offset].reset(shift); }
		void reset(BitPos pos) { reset(pos.offset, pos.shift); }
		void flip() { for (int i = 0; i < height_; i++) data_[i].flip(); }
		void flip(int offset, int shift) { data_[offset].flip(shift); }
		void flip(BitPos pos) { flip(pos.offset, pos.shift); }
	private:
		short height_;
		short width_;
		std::unique_ptr<BitRow[]> data_;
	};
}


/////////////////////////////////////////////////////
///////////////// APPLICATION LOGIC /////////////////
/////////////////////////////////////////////////////
namespace {
	using libcamera::Stream;
	using boost::property_tree::ptree;
	class Digit
	{
	public:
		Digit() {}
		std::string glyph;
		void Read(ptree const &def);
		void Expand(int factor);
		void SetHints(int offset, int shift);
		int Score(const BitMatrix &data);
		int Score(const BitMatrix &data, int offset_hint, int shift_hint);
		int FullScore(const BitMatrix &data);
		int FullScore(const BitMatrix &data, int offset_min, int offset_max, int spacing = 0);
		int FullScore(const BitMatrix &data, int offset_min, int offset_max, int shift_min, int shift_max, int spacing = 0);
		void ShowScore(const BitMatrix &data) {return ShowScore(data, last_offset, last_shift, last_spacing);}
		void ShowScore(const BitMatrix &data, int offset, int shift, int spacing);

		int digit_spacing;
		Digit *prev = NULL;
		Digit *next = NULL;
		bool verbose;

		int last_score;
		int last_offset;
		int last_shift;
		int last_spacing;
	private:
		bool print_score_ = false;
		bool has_hints_ = false;
		int n_rows_;
		int width_;
		BitMatrix data_;

		int score_area_(const BitMatrix &data, int offset, int shift);
		int score_row_(const BitRow myrow, const BitRow data, int shift);
		void show_score_(const BitMatrix &data) {return show_score_(data, last_offset, last_shift);}
		void show_score_(const BitMatrix &data, int offset, int shift);
	};

	class NumbersStage : public PostProcessingStage
	{
	public:
		NumbersStage(LibcameraApp *app) : PostProcessingStage(app) {}

		char const *Name() const override;

		void Read(ptree const &params) override;

		void Configure() override;

		bool Process(CompletedRequestPtr &completed_request) override;

		bool verbose;
	private:
		Stream *stream_;
		Digit digits_[10];
		int n_chars_;
		int h_res_;
		int char_res_;
		int factor_;
		float ftop_, fbottom_;
		float fleft_[2], fright_[2];
		int top_, bottom_;
		int left_[2], right_[2];
		int width_, height_, stride_;
		int charrows_;
		std::vector<BitMatrix> chardata_;
	};

	#define NAME "numbers"

	char const *NumbersStage::Name() const
	{
		return NAME;
	}

	void Digit::Read(ptree const &def)
	{
		n_rows_ = def.size();
		width_ = def.front().second.data().length();
		data_ = BitMatrix(n_rows_, width_);
		if (verbose)
			std::cerr << "Reading digit " << glyph << " with " << n_rows_ << " rows of length " << width_ << ":" << std::endl;

		int r = 0;
		for (auto &&[key, elem] : def)
		{
			std::string str = elem.data();
			assert(str.length() == width_);
			BitRow rval = BitRow(str, 0, str.length(), '.', '#');
			data_[r++] = rval;
			if (verbose)
				std::cerr << "- " << (rval << ((BITS - width_) / 2)) << std::endl;
		}
	}

	void Digit::Expand(int factor) {
		if (verbose)
			std::cerr << "Expanding digit " << glyph << " by " << factor << std::endl;
		BitMatrix new_data(n_rows_ * factor, width_);
		BitRow *ptr = new_data.begin();
		for (int r = 0; r < n_rows_; r++) {
			for (int i = 0; i < factor; i++) {
				*ptr++ = data_[r];
				// std::cerr << "+ " << curdata[r] << std::endl;
			}
		}
		data_ = std::move(new_data);
		n_rows_ *= factor;
	}


	// void Digit::SetHints(int offset, int shift) {
	// 	last_offset = offset;
	// 	last_shift = shift;
	// 	has_hints_ = true;
	// }

	// int Digit::Score(const BitMatrix &data) {
	// 	if (has_hints_) {
	// 		return Score(data, has_hints_ ? last_offset : (data.height() - n_rows_) / 2, has_hints_ ? last_shift : (BITS - width_) / 2);
	// 	} else {
	// 		return FullScore(data);
	// 	}
	// }

	// int Digit::Score(const BitMatrix &data, int offset_hint, int shift_hint) {
	// 	int score = score_area_(data, offset_hint, shift_hint);
	// 	return score;
	// }
	int Digit::FullScore(const BitMatrix &data) {
		return FullScore(data, -n_rows_ + 1, data.height() - 1, digit_spacing);
	}

	int Digit::FullScore(const BitMatrix &data, int offset_min, int offset_max, int spacing) {
		return FullScore(data, offset_min, offset_max, 0, BITS - width_, spacing);
	}

	int Digit::FullScore(const BitMatrix &data, int offset_min, int offset_max, int shift_min, int shift_max, int spacing) {
		int best_score = INT_MIN;
		for (int offset = offset_min; offset <= offset_max; offset++) {
			for (int shift = shift_min; shift <= shift_max; shift++) {
				int score = score_area_(data, offset, shift);
				if (score > best_score) {
					best_score = score;
					last_offset = offset;
					last_shift = shift;
				}
			}
		}
		if (spacing) {
			last_spacing = 0;
			Digit *neighbor = NULL;
			if (last_offset < 0) {
				neighbor = next;
			} else if (n_rows_ + last_offset > data.height()) {
				neighbor = prev;
				spacing = -spacing;
			}
			if (neighbor) {
				best_score += neighbor->FullScore(data, last_offset + spacing * 9/10, last_offset + spacing * 11/10, 0);
				last_spacing = neighbor->last_offset - last_offset;
			}
		}
		last_score = best_score;
		return best_score;
	}

	int Digit::score_area_(const BitMatrix &data, int offset, int shift) {
		const BitRow *myrow = data_.begin();
		const BitRow *datarow = data.begin();
		int myrows = n_rows_;
		int datarows = data.height();
		int score = 0;
		if (offset <= 0) {
			// digit is above window
			myrow -= offset;
			myrows += offset;
		} else {
			datarow += offset;
			datarows -= offset;
		}
		int count = std::min(myrows, datarows);
		// std::cerr << "score " << datarows << " rows @"<<shift<<","<<offset<<std::endl;
		while (count-- > 0) {
			score += score_row_(*myrow++, *datarow++, shift);
		}
		return score;
	}

	int Digit::score_row_(const BitRow myrow, const BitRow data, int shift) {
		BitRow shiftrow = myrow << shift;
		BitRow xorrow = shiftrow ^ data;
		int expectednotfound = (shiftrow & xorrow).count();
		int expectedandfound = (shiftrow & ~xorrow).count();
		// int score = BITS - xorrow.count() - expectednotfound * 4 + expectedandfound;
		int score = expectedandfound - expectednotfound * 2;
		if (print_score_) {
			std::cout << "  ";
			for (int i = BITS - 1; i >= 0; i--) {
				if (!xorrow.test(i)) {
					// match in this position
					std::cout << (shiftrow.test(i) ? "#######" : ".......");
				} else if (shiftrow.test(i)) {
					// expected pixel in this position, not found
					std::cout << "xxxxxxx";
				} else {
					// expected blank in this position, found pixel
					std::cout << "_______";
				}
			}
			// std::cout << "  " << shiftrow << " ^ " << data << " = " << xorrow << " (" << score << ")" << std::endl;
			// printf(" (%d = %d - %d - %d * 4 + %d)\n", score, BITS, (int)xorrow.count(), expectednotfound, expectedandfound);
			printf(" (%d = %d - %d * 2)\n", score, expectedandfound, expectednotfound);
			// std::cout << " (" << score << ")" << std::endl;
		}
		return score;
	}

	void Digit::ShowScore(const BitMatrix &data, int offset, int shift, int spacing) {
		if (spacing < -digit_spacing / 2) {
			prev->FullScore(data, offset + spacing, offset + spacing, 0);
			prev->show_score_(data);
			std::cout << std::endl;
		}
		show_score_(data, offset, shift);
		if (last_spacing > digit_spacing / 2) {
			std::cout << std::endl;
			next->FullScore(data, offset + spacing, offset + spacing, 0);
			next->show_score_(data);
		}
	}

	void Digit::show_score_(const BitMatrix &data, int offset, int shift) {
		print_score_ = true;
		score_area_(data, offset, shift);
		print_score_ = false;
	}

	void NumbersStage::Read(ptree const &params)
	{
		verbose = params.get<bool>("verbose", false);
		n_chars_ = params.get<int>("n_chars", 1);
		h_res_ = params.get<int>("h_res", 16);
		if (h_res_ < 1 || h_res_ > 32)
			throw std::runtime_error("NumbersStage: h_res must be between 1 and 32");
		ftop_ = params.get<float>("top", 0.0);
		fbottom_ = params.get<float>("bottom", 1.0);

		float left = params.get<float>("left", 0.0);
		float right = params.get<float>("right", 1.0);

		fleft_[0] = params.get<float>("left.top", left);
		fleft_[1] = params.get<float>("left.bottom", left);
		fright_[0] = params.get<float>("right.top", right);
		fright_[1] = params.get<float>("right.bottom", right);
		if (verbose)
			printf("n_chars = %d\nh_res = %d\ntop = %f\nbottom = %f\nleft = %f,%f\nright = %f,%f\n", n_chars_, h_res_, ftop_, fbottom_, fleft_[0], fleft_[1], fright_[0], fright_[1]);

		for (int i = 0; i < 10; i++) {
			digits_[i].verbose = verbose;
			digits_[i].next = digits_ + (i + 1) % 10;
			digits_[i].prev = digits_ + (i + 9) % 10;
		}

		char_res_ = params.get<int>("char_res", h_res_);
		for (auto &&cdata : params.get_child("chars")) {
			auto cname = cdata.first;
			auto cdef = cdata.second;
			int n = -1;
			if (cname.length() == 1) {
				n = cname.at(0) - '0';
			}
			if (n < 0 || n > 9) {
				throw std::runtime_error("NumbersStage: chars only supports 0 through 9");
			}
			digits_[n].glyph = cname;
			digits_[n].Read(cdef);
		}
	}

	void NumbersStage::Configure()
	{
		stream_ = app_->GetMainStream();
		if (!stream_ || stream_->configuration().pixelFormat != libcamera::formats::YUV420)
			throw std::runtime_error("NumbersStage: only YUV420 format supported");
		auto config = stream_->configuration();
		auto pixfmt = config.pixelFormat;
		width_ = config.size.width;
		height_ = config.size.height;
		stride_ = config.stride;
		left_[0] = fleft_[0] * width_;
		left_[1] = fleft_[1] * width_;
		right_[0] = fright_[0] * width_;
		right_[1] = fright_[1] * width_;
		top_ = ftop_ * height_;
		bottom_ = fbottom_ * height_;
		charrows_ = bottom_ - top_;
		chardata_.resize(n_chars_);
		for (BitMatrix &cdata : chardata_) {
			cdata = BitMatrix(charrows_);
		}
		
		factor_ = (right_[0] - left_[0]) / n_chars_ / h_res_;
		if (verbose)
			printf("size: %s, stride: %d, frameSize: %d, pixfmt: %s, factor: %d\n", config.size.toString().c_str(), config.stride, config.frameSize, pixfmt.toString().c_str(), factor_);
		for (int n = 0; n < 10; n++) {
			digits_[n].Expand(factor_);
			digits_[n].digit_spacing = charrows_;
		}
	}

	bool NumbersStage::Process(CompletedRequestPtr &completed_request)
	{
		libcamera::Span<uint8_t> buffer = app_->Mmap(completed_request->buffers[stream_])[0];
		int rows = charrows_;
		uint8_t *row = buffer.data() + (top_ * stride_);

		for (BitMatrix &cdata : chardata_) {
			cdata.reset();
		}

		if (verbose)
			printf("n_chars = %d\ntop = %d\nbottom=%d\nleft=%d,%d\nright=%d,%d\n", n_chars_, top_, bottom_, left_[0], left_[1], right_[0], right_[1]);

		// y = top;
		// for (x = left_[0]; x < right_[0]; x++)
		// 	row[x] = 255;

		for (int r = 0; r < rows; r++) {
			row += stride_;

			int left = left_[0] + (left_[1] - left_[0]) * r / rows;
			int right = right_[0] + (right_[1] - right_[0]) * r / rows;
			int cwidth = (right - left) / n_chars_;
			int pwidth = cwidth / h_res_;

			for (int i = 0; i < n_chars_; i++) {
				int cleft = left + (right - left) * i / n_chars_ + pwidth / 2;
				for (int p = 0; p < h_res_; p++) {
					bool bit = row[cleft + p * pwidth] == 0xFF;
					chardata_[i].set(r, h_res_ - p - 1, bit);
					row[p + i*h_res_] = bit ? 0xFF : 0;
				}
				row[cleft - pwidth / 2] = 0xFF;
			}

			row[right] = 0xFF;
		}

		// ptr += stride_;
		// for (x = left_[1]; x < right_[1]; x++)
		// 	*ptr++ = 255;

		for (int i = 0; i < n_chars_; i++) {
			int best_score = 0;
			Digit *best_digit = NULL;
			BitMatrix &chardata = chardata_[i];
			if (verbose) {
				std::cout << "Char " << i << ":" << std::endl;
				for (int r = 0; r < rows; r += factor_) {
					std::cout << chardata[r] << std::endl;
				}
				std::cout << "Scoring:" << std::endl;
			}
			for (Digit &dig : digits_) {
				int score = dig.FullScore(chardata);
				if (score > best_score) {
					best_score = score;
					best_digit = &dig;
				}
				if (verbose) {
					std::cout << " digit " << dig.glyph << " score: " << score;
					if (dig.last_spacing) {
						int neighbor_score = (dig.last_spacing < 0 ? dig.prev : dig.next)->last_score;
						std::cout << " (" << (score - neighbor_score) << " + " << neighbor_score << ")";
					}
					std::cout << " @" << dig.last_shift << "," << dig.last_offset;
					if (dig.last_spacing) {
						printf(" (%+d)", dig.last_spacing);
					}
					std::cout << std::endl;
				}
			}
			auto &&bd = *best_digit;
			if (verbose) {
				std::cout << "best guess: " << bd.glyph << " @" << bd.last_shift << "," << bd.last_offset << "/" << (bd.last_offset / factor_) << std::endl;
				bd.ShowScore(chardata);
				std::cout << std::endl;
			} else {
				std::cout << bd.glyph;
			}
		}

		if (verbose)
			printf("done\n");
		else
			std::cout << std::endl;

		return false;
	}

	static PostProcessingStage *Create(LibcameraApp *app)
	{
		return new NumbersStage(app);
	}

	static RegisterStage reg(NAME, &Create);
}