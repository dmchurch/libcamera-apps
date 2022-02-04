#include <libcamera/stream.h>
#include <bitset>

#include "core/libcamera_app.hpp"

#include "post_processing_stages/post_processing_stage.hpp"

#define BITS 32

inline namespace {
	using Stream = libcamera::Stream;
	using ptree = boost::property_tree::ptree;
	using bitset = std::bitset<BITS>;

	struct BitPos
	{
		short offset;
		short shift;

		BitPos(): offset(SHRT_MIN), shift(SHRT_MIN) { }
		BitPos(short offset, short shift): offset(offset), shift(shift) { }
	};
	struct Score: public BitPos
	{
		int score;

		Score(): score(INT_MIN) { }
		Score(int score, short offset, short shift): BitPos(offset, shift), score(score) { }

		virtual int TotalScore() const {
			return score;
		}

		operator bool() const { return score != INT_MIN; }

		bool operator>(const Score &&rhs) const { return TotalScore() > rhs.TotalScore(); }
		bool operator>=(const Score &&rhs) const { return TotalScore() >= rhs.TotalScore(); }
		bool operator<(const Score &&rhs) const { return TotalScore() < rhs.TotalScore(); }
		bool operator<=(const Score &&rhs) const { return TotalScore() <= rhs.TotalScore(); }
		bool operator==(const Score &&rhs) const { return TotalScore() == rhs.TotalScore(); }
		bool operator!=(const Score &&rhs) const { return TotalScore() != rhs.TotalScore(); }

		friend std::ostream &operator<<(std::ostream &os, Score score) {
			return os << score.TotalScore() << " @" << score.shift << "," << score.offset;
		}
	};


	struct CombinedScore: Score {
		Score neighbor_score;
		CombinedScore() : Score() { }
		explicit CombinedScore(Score score) : Score(score) { }
		CombinedScore(Score score, Score neighbor_score): Score(score), neighbor_score(neighbor_score) { }
		CombinedScore(int score, short offset, short shift, Score neighbor_score): Score(score, offset, shift), neighbor_score(neighbor_score) { }
		CombinedScore(int score, short offset, short shift, int neighbor_score, short neighbor_offset, short neighbor_shift): Score(score, offset, shift), neighbor_score(neighbor_score, neighbor_offset, neighbor_shift) { }

		int TotalScore() const override {
			return score + (neighbor_score ? neighbor_score.score : 0);
		}

		int spacing() const { return neighbor_score ? neighbor_score.offset - offset : 0; }
	};

	std::ostream &operator <<(std::ostream &os, CombinedScore score) {
		os << score.TotalScore();
		if (score.neighbor_score) {
			// int neighbor_score = (dig.last_spacing < 0 ? dig.prev : dig.next)->last_score;
			os << " (" << score.score << " + " << score.neighbor_score.score << ")";
		}
		os << " @" << score.shift << "," << score.offset;
		if (score.neighbor_score) {
			os << " (" << std::showpos << score.spacing() << ")" << std::endl;
			// printf(" (%+d)", dig.last_spacing);
		}

		return os;
	}

	struct LocalMaximum {
		Score score;
		short offset_min;
		short offset_max;
	};

	using BitRow = bitset;
	// class BitRow: public bitset {
	// public:
	// 	BitRow(bitset &&bits) : bitset(bits) { }
	// };

	// template <class _CharT, class _Traits>
	// std::basic_ostream<_CharT, _Traits>&
	// operator<<(std::basic_ostream<_CharT, _Traits>& __os, const BitRow& __x)
	// {
	// 	std::basic_string<_CharT, _Traits> __tmp;

	// 	__tmp.assign(BITS, '.');

	// 	for (int b = BITS - 1, i = 0; b >= 0; b--, i++) {
	// 		if (__x.test(b))
	// 			__tmp[i] = '#';
	// 	}

	// 	return __os << __tmp;
	// }

	/// BitRange: a range of offset/shift values in the global coordinate system
	class BitRange {
	public:
		BitRange() :BitRange(0) { }
		BitRange(int height, int width = BITS, int offset = 0, int shift = 0) :height_(height), width_(width), offset_(offset), shift_(shift) { }

		unsigned int height() const { return height_; }
		unsigned int width() const { return width_; }
		int offset() const { return offset_; }
		int shift() const { return shift_; }

		int offset_min() const { return -offset_; }
		int offset_max() const { return height_ - offset_ - 1; }

		int bit_min() const { return shift_; }
		int bit_max() const { return width_ + shift_ - 1; }

		BitRange &height(int height) { height_ = height; return *this; }
		BitRange &width(int width) { width_ = width; return *this; }

		bool contains(BitPos pos) const { return offset_min() <= pos.offset
										&& offset_max() >= pos.offset
										&& bit_min() <= pos.shift
										&& bit_max() >= pos.shift; }
		BitRange origin() { shift_ = offset_ = 0; return *this; }
		BitRange center() { shift_ = ((BITS - width_) / 2); return *this; }
		BitRange fullwidth() { shift_ = 0; width_ = BITS; return *this; }

		operator bool() const { return height_ > 0 && width_ > 0; }

		bool operator> (BitRange other) { return offset_min() > other.offset_max(); }
		bool operator>=(BitRange other) { return offset_min() >= other.offset_min(); }
		bool operator< (BitRange other) { return offset_max() < other.offset_min(); }
		bool operator<=(BitRange other) { return offset_max() <= other.offset_max(); }

		// int +, -, <<, >> arithmetic: shift this range in the window
		BitRange &operator+=(int o) { offset_ += o; return *this; }
		BitRange &operator-=(int o) { offset_ -= o; return *this; }
		BitRange &operator<<=(int s) { shift_ += s; return *this; }
		BitRange &operator>>=(int s) { shift_ -= s; return *this; }

		template<typename TBits> friend std::enable_if_t<std::is_base_of_v<BitRange, TBits>, TBits>
		operator+(const TBits range, int o) { return TBits(range) + o; }
		template<typename TBits> friend std::enable_if_t<std::is_base_of_v<BitRange, TBits>, TBits>
		operator-(const TBits range, int o) { return TBits(range) - o; }
		template<typename TBits> friend std::enable_if_t<std::is_base_of_v<BitRange, TBits>, TBits>
		operator<<(const TBits range, int s) { return TBits(range) << s; }
		template<typename TBits> friend std::enable_if_t<std::is_base_of_v<BitRange, TBits>, TBits>
		operator>>(const TBits range, int s) { return TBits(range) >> s; }

		// BitRange +, -, *, %: union, asymmetric difference, intersection, symmetric difference
		BitRange &operator+=(const BitRange rhs);
		BitRange &operator-=(const BitRange rhs);
		BitRange &operator*=(const BitRange rhs);
		BitRange &operator%=(const BitRange rhs);

	protected:
		// this class can be stored as a single 64-bit word
		unsigned short height_;
		unsigned short width_;
		short offset_; // in the + direction, which shifts UP, or as though incrementing a pointer into a dataset
		short shift_; // in the << direction, which shifts RIGHT, since lsb is on the left
	};
	static_assert(sizeof(BitRange) == 8, "BitRange exceeds 64-bit size");

	template<class TDataSrc, class TData = typename TDataSrc::Data> class BitSource;
	class BitGrid;

	template<typename T>
	struct is_bit_source : std::false_type {};

	template<typename T>
	struct is_bit_source<BitSource<T>> : std::true_type {
		using type = T;
	};

	template<typename T> inline constexpr bool is_bit_source_v = is_bit_source<T>::value;
	template<typename T> using bit_source_t = typename is_bit_source<T>::type;

	/// BitSource: a BitRange that also knows how to access some data for its range
	/// @tparam TDataSrc a type describing the source of the bit data
	template <class TDataSrc, class TData>
	class BitSource: public BitRange {
	public:
		// get a ref to the bitset representing at a given global offset, unshifted
		bitset &operator[](int offset) & { return data_[offset + offset_]; }
		// shorthand for (*this)[0]
		bitset &operator*() & { return (*this)[0]; }

		// get the row located at a given global offset, shifted
		BitRow row_at(int offset) const { return (*this)[offset] << shift_; }

		// materialize all the rows represented by this source into an allocated table
		BitGrid materialize() const;

		operator BitGrid() const;

		// BitSource operator+(int o) const { return BitSource(*this) += o; }
		// BitSource operator-(int o) const { return BitSource(*this) -= o; }
		// BitSource operator<<(int s) const { return BitSource(*this) <<= s; }
		// BitSource operator>>(int s) const { return BitSource(*this) >>= s; }

		// BitSource &, |, ^, ~: bitwise manips
		template<typename TRhs>
		auto operator&(const TRhs rhs) { return BitOp<std::bit_and<bitset>, TRhs>(*this, rhs); }
		template<typename TRhs>
		auto operator|(const TRhs rhs) { return BitOp<std::bit_or<bitset>, TRhs>(*this, rhs); }
		template<typename TRhs>
		auto operator^(const TRhs rhs) { return BitOp<std::bit_xor<bitset>, TRhs>(*this, rhs); }
		auto operator~() { return BitOp<std::bit_not<bitset>>(*this); }

		// to access formatting operators
		class BitFormatter;
		BitFormatter fmt() const;

		/// binary BitOp - binary operation BitSource
		/// @tparam TOp Standard operation on bitset
		/// @tparam TRhs type of right-hand BitSource operand
		template<class TOp, class TRhs = void, typename = std::enable_if_t<(std::is_void_v<TRhs> || is_bit_source_v<TRhs>)>>
		class BitOp;
		/// unary BitOp - unary operation BitSource
		/// @tparam TOp Standard operation on bitset
		template<class TOp>
		class BitOp<TOp, void>;


	protected:
		BitSource() :BitRange() { }
		BitSource(BitRange range) :BitRange(range) { }
		BitSource(BitRange range, TData data) :BitRange(range), data_(data) { }
		BitSource(BitRange range, TDataSrc data) :BitRange(range), data_(data._data) { }
		// BitSource(BitSource &&) = delete;
		TData data_;
	};

	/// BitGrid: a BitSource that stores and can modify mutable bits
	class BitGrid: public BitSource<BitGrid> {
	public:
		using Data = std::shared_ptr<std::vector<bitset>>;
		BitGrid() :BitSource() { }
		BitGrid(int height, int width = BITS) :BitSource(BitRange{height, width}) { }
		BitGrid(BitRange range) :BitSource(range) { }
		BitGrid(BitRange range, BitGrid data) :BitSource(range, data) { }

		BitGrid &set();
		BitGrid &set(int row, int bit);
		BitGrid &set(int row, int bit, bool value);
		BitGrid &reset();
		BitGrid &reset(int row, int bit);
		BitGrid &flip();
		BitGrid &flip(int row, int bit);

		size_t count() const;

		BitGrid expand(int factor) const;

		BitRange intersection(const BitGrid rhs) const;
		bool contains(const BitRange other) const;
		BitGrid at(BitRange range) { return BitGrid(range, *this); }

		// BitGrid Centered() const { return *this << ((BITS - width_) / 2); }

		// const BitRow operator*() const { return (*this)[0]; }

		// const BitRow operator[](int i) const { return materialize(i + offset_) << shift_; }
		// const BitRow row_at(int offset) const { return materialize() }
		// bitset &&operator[](int i) {
		// 	if (shift_) throw std::runtime_error("cannot return lvalue to shifted bitset");
		// 	return materialize(i + offset_);
		// }

		// Delayed-evaluation grid operators
		// template<typename TFunc, typename TLhs, typename TRhs = void>
		// class Op: public BitRange { //public std::enable_if_t<std::is_base_of_v<BitGrid, TLhs>>, BitGrid> {
		// 	using OpFunc = TFunc();
		// public:
		// 	Op(TLhs op1, TRhs op2): op1_(op1), op2_(op2) { }
		// 	operator BitGrid() const;
		// protected:
		// 	virtual bitset materialize(int offset) const override;
		// private:
		// 	TLhs op1_;
		// 	TRhs op2_;
		// };

		// // int +, -, <<, >> arithmetic: shift this grid in the window
		// BitGrid &operator+=(int o) { offset_ += o; return *this; }
		// BitGrid &operator-=(int o) { offset_ -= o; return *this; }
		// BitGrid &operator<<=(int s) { shift_ += s; return *this; }
		// BitGrid &operator>>=(int s) { shift_ -= s; return *this; }

		// BitGrid operator+(int o) const { return BitGrid(*this) += o; }
		// BitGrid operator-(int o) const { return BitGrid(*this) -= o; }
		// BitGrid operator<<(int s) const { return BitGrid(*this) <<= s; }
		// BitGrid operator>>(int s) const { return BitGrid(*this) >>= s; }

		// // BitGrid &, |, ^, ~: bitwise manips
		// template<typename TLhs, typename TRhs>
		// friend auto operator&(const TLhs lhs, const TRhs rhs) { return Op<std::bit_and<bitset>, TLhs, TRhs>(lhs, rhs); }
		// template<typename TLhs, typename TRhs>
		// friend auto operator|(const TLhs lhs, const TRhs rhs) { return Op<std::bit_or<bitset>, TLhs, TRhs>(lhs, rhs); }
		// template<typename TLhs, typename TRhs>
		// friend auto operator^(const TLhs lhs, const TRhs rhs) { return Op<std::bit_xor<bitset>, TLhs, TRhs>(lhs, rhs); }
		// template<typename TLhs, typename TRhs>
		// friend auto operator~(const TLhs lhs) { return Op<std::bit_not<bitset>, TLhs>(lhs); }

		bool operator==(const BitGrid rhs) const;

		// to access formatting operators
		BitFormatter fmt() const;

	protected:
		// void check_materialization() const {
		// 	if (!is_materialized())
		// 		const_cast<BitGrid *>(this)->materialize(); // doesn't affect external view of object
		// }
		// virtual bool is_materialized() const {
		// 	return rows_ && height() == rows_->size();
		// }
		// bitset materialize(int offset) const {
		// 	if (!rows_) {
		// 		const_cast<decltype(rows_) &>(rows_) = std::make_shared<std::vector<bitset>>(height());
		// 	}
		// 	if (height() != rows_->size())
		// 		rows_->resize(height());
		// 	return (*rows_)[offset];
		// }
		// std::shared_ptr<std::vector<bitset>> rows_;

		// friend class BitFormatter;
	};

	template<class TDataSrc>
	template<class TOp, class TRhs, typename>
	class BitSource<TDataSrc>::BitOp : BitSource<BitOp> {
		using TLhs = BitSource<TDataSrc>;
		static inline constexpr auto OpFunc = TOp().operator();
	public:
		BitOp(TLhs &&lhs, TRhs &&rhs) :BitSource((lhs * rhs).fullwidth(), {lhs, rhs}) { }
		operator[](int offset) {
			return OpFunc(data_.lhs.row_at(offset), data_.rhs.row_at(offset));
		}
	protected:
		struct Data {
			Data(TLhs &&lhs, TRhs &&rhs): lhs_(lhs), rhs_(rhs) { }
			TLhs &&lhs_;
			TRhs &&rhs_;
		}
	};

	template<class TDataSrc>
	template<class TOp>
	class BitSource<TDataSrc>::BitOp<TOp> {
		using TOperand = BitSource<TDataSrc>;
		using Data = TOperand;
		static inline constexpr auto OpFunc = TOp().operator();
	public:
		BitOp(TOperand &&operand) :BitSource(BitRange(operand).fullwidth(), operand) { }
		operator[](int offset) {
			return OpFunc(data_.row_at(offset));
		}
	};

	template<class TDataSrc>
	class BitSource<TDataSrc>::BitFormatter {
	public:
		BitFormatter(BitGrid grid): grid_(grid) { }

		BitFormatter Indented(std::string prefix) {
			prefix_ += prefix;
			return *this;
		}

		BitFormatter Stretched(int factor) {
			stretch_ = factor;
			return *this;
		}

		// no, these aren't opposites, because I hate myself
		BitFormatter Contracted(int factor) {
			contract_ = factor;
			return *this;
		}

		BitFormatter Centered() {
			grid_ <<= ((BITS - grid_.width()) / 2) - grid_.shift();
			return *this;
		}

		void print(std::ostream &os);
		friend std::ostream &operator<<(std::ostream &os, BitFormatter bf) { bf.print(os); return os; }
	private:
		BitGrid grid_;
		std::string prefix_;
		int stretch_;
		int contract_;
	};
	
	template <class TDataSrc>
	BitSource<TDataSrc>::operator BitGrid() const { return materialize(); }
	template <class TDataSrc>
	BitSource<TDataSrc>::BitFormatter BitSource<TDataSrc>::fmt() const { return materialize().fmt(); }
	template<class TDataSrc>
	std::ostream &operator<<(std::ostream &os, BitSource<TDataSrc> grid) { return os << grid.fmt(); }
	BitGrid::BitFormatter BitGrid::fmt() const { return BitFormatter(*this); }

	class Digit
	{
	public:
		Digit() {}
		std::string glyph;
		void Read(ptree const &def);
		void Expand(int factor);
		// void SetHints(int offset, int shift);
		// Score CalcScore(const BitGrid data);
		// Score CalcScore(const BitGrid data, int offset_hint, int shift_hint);
		CombinedScore FullScore(const BitGrid data);
		CombinedScore FullScore(const BitGrid data, int offset_min, int offset_max, int spacing = 0);
		CombinedScore FullScore(const BitGrid data, int offset_min, int offset_max, int shift_min, int shift_max, int spacing = 0);
		void ShowScore(const BitGrid data) { ShowScore(data, last_score); }
		void ShowScore(const BitGrid data, CombinedScore score);

		int digit_spacing;
		Digit *prev = NULL;
		Digit *next = NULL;
		bool verbose;

		// int last_spacing;
	private:
		CombinedScore last_score;
		bool print_score_ = false;
		bool has_hints_ = false;
		// int width_;
		BitGrid match_grid_;

		Score score_area_(const BitGrid data, int offset, int shift);
		int score_row_(const bitset myrow, const bitset data, int shift);
		void show_score_(const BitGrid data) { return show_score_(data, last_score); }
		void show_score_(const BitGrid data, Score score);
	};

	std::ostream &operator<<(std::ostream &os, Digit digit) {
		return os << digit.glyph;
	}

	#define NAME "numbers"

	class NumbersStage : public PostProcessingStage
	{
	public:
		NumbersStage(LibcameraApp *app) : PostProcessingStage(app) {}

		char const *Name() const override { return NAME; }

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
		std::unique_ptr<BitGrid[]> chardata_;
	};

	void Digit::Read(ptree const &def)
	{
		match_grid_ = BitGrid(def.size());
		int width = 0;
		if (verbose)
			std::cerr << "Reading digit " << glyph << " with " << match_grid_.height() << " rows:" << std::endl;

		int r = 0;
		for (auto &&row : def)
		{
			bitset rval = 0;
			std::string str = row.second.get_value<std::string>();
			width = std::max(width, (int)str.length());
			for (unsigned int i = 0; i < str.length(); i++) {
				if (str.at(i) == '#')
					rval.set(i);
			}
			match_grid_[r++] = rval;
		}
		match_grid_.width(width);
		if (verbose)
			std::cerr << match_grid_.fmt().Centered().Indented("- ") << std::endl;
	}

	void Digit::Expand(int factor) {
		if (verbose)
			std::cerr << "Expanding digit " << glyph << " by " << factor << std::endl;
		match_grid_.expand(factor);
		// bitset *curdata = match_grid_;
		// match_grid_ = new bitset[match_grid_.size() * factor];
		// bitset *ptr = match_grid_;
		// for (int r = 0; r < match_grid_.size(); r++) {
		// 	for (int i = 0; i < factor; i++) {
		// 		*ptr++ = curdata[r];
		// 		// std::cerr << "+ " << curdata[r] << std::endl;
		// 	}
		// }
		// n_rows_ *= factor;
		// delete curdata;
	}


	// void Digit::SetHints(int offset, int shift) {
	// 	last_score = {0, offset, shift};
	// 	// last_offset = offset;
	// 	// last_shift = shift;
	// 	has_hints_ = true;
	// }

	// Score Digit::CalcScore(const bitset data[], int rows) {
	// 	if (has_hints_) {
	// 		return CalcScore(data, rows, last_offset, last_shift;
	// 	} else {
	// 		return FullScore(data, rows);
	// 	}
	// }

	// Score Digit::CalcScore(const bitset data[], int rows, int offset_hint, int shift_hint) {
	// 	int score = score_area_(data, rows, offset_hint, shift_hint);
	// 	return score;
	// }
	CombinedScore Digit::FullScore(const BitGrid data) {
		return FullScore(data, -match_grid_.offset_max(), data.offset_max(), digit_spacing);
	}

	CombinedScore Digit::FullScore(const BitGrid data, int offset_min, int offset_max, int spacing) {
		return FullScore(data, offset_min, offset_max, 0, BITS - match_grid_.width(), spacing);
	}

	CombinedScore Digit::FullScore(const BitGrid data, int offset_min, int offset_max, int shift_min, int shift_max, int spacing) {
		last_score = {};
		CombinedScore best_score = last_score = {};
		// int best_offset = last_offset = INT_MIN;
		// int best_shift = last_shift = INT_MIN;
		Score current_score = best_score;
		Score neighbor_score;
		// int /* neighbor_score, neighbor_offset, */ neighbor_test_min, neighbor_test_max;
		BitRange neighbor_test;
		int spacing_min = spacing * 9/10;
		int spacing_max = spacing * 11/10;

		// neighbor_offset = INT_MIN;
		for (int offset = offset_min; offset <= offset_max; offset++) {
			for (int shift = shift_min; shift <= shift_max; shift++) {
				current_score = std::max(current_score, score_area_(data, offset, shift));
			}
			if (current_score != best_score) { // current_score.offset == offset) {
				// we just came up with a new best-offset; should we check the neighbors?
				Digit *test_neighbor = NULL;
				bool keep_neighbor_score = false;
				auto test_range = BitRange(spacing_max - spacing_min) + offset; // centered on this location
				// int test_min, test_max;
				if (match_grid_.offset_min() < data.offset_min() && spacing && next) {
					// this glyph extends above the region of interest, check the next neighbor as well
					test_neighbor = next;
					test_range += spacing_min; // move down 
					// test_min = offset + spacing_min;
					// test_max = offset + spacing_max;
				} else if (match_grid_.offset_max() > data.offset_max() && spacing && prev) {
					// this glyph extends below the ROI, check the previous neighbor
					test_neighbor = prev;
					test_range -= spacing_max; // move up
					// test_min = offset - spacing_max;
					// test_max = offset - spacing_min;
				}
				if (test_neighbor) {
					// run the neighbor test (on the fewest rows possible)
					if (test_range.contains(neighbor_score)) {
						// we have a valid pre-existing neighbor score, don't retest valid ranges
						keep_neighbor_score = true;
						test_range -= neighbor_test;
						// test_min = neighbor_test_max + 1;
					}
					Score new_neighbor_score = test_neighbor->FullScore(data, test_range.offset_min(), test_range.offset_max(), 0);
					if (keep_neighbor_score && new_neighbor_score < neighbor_score) {
						// the new neighbor score is worse than the current. discard it, but update the test range.
						neighbor_test += test_range;
					} else {
						// record the new score and the testing range.
						neighbor_score = new_neighbor_score;
						neighbor_test = test_range;
					}
				}
			}
		}
		// if (spacing) {
		// 	last_spacing = 0;
		// 	Digit *neighbor = NULL;
		// 	if (last_score.offset < 0) {
		// 		neighbor = next;
		// 	} else if (match_grid_.size() + last_score.offset > data.offset_max()) {
		// 		neighbor = prev;
		// 		spacing = -spacing;
		// 	}
		// 	if (neighbor) {
		// 		best_score += neighbor->FullScore(data, last_score.offset + spacing * 9/10, last_score.offset + spacing * 11/10, 0);
		// 		last_spacing = neighbor->last_score.offset - last_offset;
		// 	}
		// }
		last_score = best_score;
		return best_score;
	}

	Score Digit::score_area_(const BitGrid data, int offset, int shift) {
		const BitGrid mydata = (match_grid_ + offset) << shift;
		auto xorgrid = mydata ^ data;
		// int expectednotfound = (mydata & xorgrid).count();
		// int expectedandfound = (mydata & ~xorgrid).count();
		// int score = expectedandfound - expectednotfound * 2;
		int score = 0;
		return Score(score, offset, shift);
		// // int myrows = match_grid_.size();
		// int score = 0;
		// if (offset <= 0) {
		// 	// digit is above window
		// 	mydata -= offset;
		// 	// myrows += offset;
		// } else {
		// 	data += offset;
		// 	// rows -= offset;
		// }
		// // int min_off = std::max(mydata.offset_min(), data.offset_min());
		// // int max_off = std::min(mydata.offset_max(), data.offset_max());
		// int count = std::min(mydata.offset_max(), data.offset_max());
		// // std::cerr << "score " << rows << " rows @"<<shift<<","<<offset<<std::endl;
		// while (count-- > 0) {
		// 	score += score_row_(*myrow++, *data++, shift);
		// }
		// return score;
	}

	// int Digit::score_row_(const bitset myrow, const bitset data, int shift) {
	// 	bitset shiftrow = myrow << shift;
	// 	bitset xorrow = shiftrow ^ data;
	// 	int expectednotfound = (shiftrow & xorrow).count();
	// 	int expectedandfound = (shiftrow & ~xorrow).count();
	// 	// int score = BITS - xorrow.count() - expectednotfound * 4 + expectedandfound;
	// 	int score = expectedandfound - expectednotfound * 2;
	// 	if (print_score_) {
	// 		std::cout << "  ";
	// 		for (int i = 0; i < BITS; i++) {
	// 			if (!xorrow.test(i)) {
	// 				// match in this position
	// 				std::cout << (shiftrow.test(i) ? "#######" : ".......");
	// 			} else if (shiftrow.test(i)) {
	// 				// expected pixel in this position, not found
	// 				std::cout << "xxxxxxx";
	// 			} else {
	// 				// expected blank in this position, found pixel
	// 				std::cout << "_______";
	// 			}
	// 		}
	// 		// std::cout << "  " << shiftrow << " ^ " << data << " = " << xorrow << " (" << score << ")" << std::endl;
	// 		// printf(" (%d = %d - %d - %d * 4 + %d)\n", score, BITS, (int)xorrow.count(), expectednotfound, expectedandfound);
	// 		printf(" (%d = %d - %d * 2)\n", score, expectedandfound, expectednotfound);
	// 		// std::cout << " (" << score << ")" << std::endl;
	// 	}
	// 	return score;
	// }

	void Digit::ShowScore(const BitGrid data, CombinedScore score) {
		// FullScore(data, score.offset, score.offset, score.shift, score.shift, digit_spacing);
		if (score.spacing() < 0) {
			prev->show_score_(data, score.neighbor_score);
			std::cout << std::endl;
		}
		show_score_(data, score);
		if (score.spacing() > 0) {
			std::cout << std::endl;
			next->show_score_(data, score.neighbor_score);
		}
	}

	void Digit::show_score_(const BitGrid data, Score score) {
		print_score_ = true;
		score_area_(data, score.offset, score.shift);
		print_score_ = false;
	}
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
	// chardata_ = new bitset[n_chars_ * charrows_];
	chardata_ = std::make_unique<BitGrid[]>(n_chars_); // std::unique_ptr<BitGrid>(new BitGrid[n_chars_]);
	for (int i = 0; i < n_chars_; i++) {
		chardata_[i] = BitGrid(charrows_);
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

	for (int i = 0; i < n_chars_; i++) {
		chardata_[i].reset();
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
				chardata_[i].set(r, p, bit);
				if (verbose) // TODO: make a new flag for this
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
		CombinedScore best_score;
		Digit *best_digit = NULL;
		BitGrid chardata = chardata_[i];
		if (verbose) {
			std::cout << "Char " << i << ":" << std::endl;
			std::cout << chardata.fmt().Contracted(h_res_) << std::endl;
			// for (int r = 0; r < rows; r += factor_) {
			// 	for (int p = 0; p < h_res_; p++) {
			// 		int bit = chardata[r].test(p);
			// 		std::cout << (bit ? '#' : '.');
			// 	}
			// 	std::cout << std::endl;
			// }
			std::cout << "Scoring:" << std::endl;
		}
		for (Digit &dig : digits_) {
			auto score = dig.FullScore(chardata);
			if (score > best_score) {
				best_score = score;
				best_digit = &dig;
			}
			if (verbose) {
				std::cout << " digit " << dig << " score: " << score << (best_digit == &dig ? " (+)" : "") << std::endl;
			}
		}
		auto &&bd = *best_digit;
		if (verbose) {
			std::cout << "best guess: " << bd << " @" << best_score.shift << "," << best_score.offset << "/" << (best_score.offset / factor_) << std::endl;
			bd.ShowScore(chardata);
			std::cout << std::endl;
		} else {
			std::cout << bd;
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
