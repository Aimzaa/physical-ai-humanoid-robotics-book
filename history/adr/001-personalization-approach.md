# ADR-001: Personalization Approach for Chapter Content

**Status**: Accepted
**Date**: 2025-12-24

## Context

The Personalized Docusaurus Book Platform requires adapting chapter content based on user profiles (software level, hardware experience, learning depth). Two fundamental approaches were considered:

1. **Hybrid Approach**: Base content + pre-written conditional sections for each profile level
2. **LLM Regeneration**: Full content regeneration using LLMs based on user profile context

The decision needed to balance:
- Response time (< 10 seconds for personalization)
- Content quality and accuracy
- Development complexity
- Operational cost (LLM API costs)
- Content maintainability

## Decision

**Chosen Approach**: Hybrid Content Adaptation

The platform will use a hybrid approach where:
- Base chapter content is written for a general intermediate audience
- Pre-written conditional sections (admonitions) are added for each profile level
- At render time, sections matching the user's profile are conditionally displayed

**Implementation Structure**:
```markdown
# Chapter: Robot Kinematics

<!-- Base content for all readers -->
## Introduction
Kinematics is the study of motion...

<!-- Conditional sections via Docusaurus admonitions -->
:::note[beginner]
## What is a Kinematic Chain?
A kinematic chain is like a series of connected arms...
:::

:::note[intermediate]
## Forward Kinematics Equations
The DH (Denavit-Hartenberg) parameters define...
:::

:::note[advanced]
## Jacobian Analysis
The Jacobian matrix relates joint velocities...
:::
```

## Consequences

### Positive
- **Predictable Performance**: No LLM latency; content renders instantly
- **Zero Runtime Cost**: No per-request API costs; content pre-written at authoring time
- **Content Quality**: All personalized content validated at write-time, not generation-time
- **Maintainability**: Changes to content are straightforward edits to markdown files
- **Translation Ready**: Pre-written content can be translated using standard i18n workflows
- **SEO Friendly**: Static content is fully crawlable

### Negative
- **Authoring Overhead**: Authors must write 3 versions of each section
- **Content Bloat**: Book size increases with conditional sections
- **Limited Flexibility**: Cannot dynamically adapt to user questions within content
- **Maintenance Burden**: Updates must be applied to all variants

## Alternatives Considered

### Alternative 1: Full LLM Regeneration

Generate personalized content on-demand using LLMs with user context in the prompt.

**Pros**:
- Unlimited flexibility and personalization depth
- Smaller book size (no content duplication)
- Can adapt to specific user questions

**Cons**:
- Variable response time (10-60 seconds typical)
- Per-request API costs multiply with users
- Risk of hallucination or inconsistency
- No built-in validation of personalized content
- Difficult to maintain tone/voice consistency

### Alternative 2: Server-Side Template Rendering

Use a template engine with profile variables substituted at build/render time.

**Pros**:
- Faster than LLM but more flexible than pre-written
- Smaller content footprint than full pre-writing

**Cons**:
- Requires build pipeline changes for Docusaurus
- Limited personalization granularity
- Still requires conditional content authoring

## References

- [Plan: specs/003-personalized-book-auth/plan.md](specs/003-personalized-book-auth/plan.md)
- [Research: specs/003-personalized-book-auth/research.md](specs/003-personalized-book-auth/research.md)
- [Feature Specification: specs/003-personalized-book-auth/spec.md](specs/003-personalized-book-auth/spec.md)

## Implementation Notes

The hybrid approach is implemented using Docusaurus's native admonition feature:

1. **Content Authors**: Write base content + conditional sections using `:::note[level]` syntax
2. **Personalization Component**: React component reads user profile from context
3. **Toggle Behavior**: User can enable/disable personalization via localStorage
4. **Default State**: Non-authenticated users see base content only

**Success Metrics**:
- Personalization response < 10 seconds (achievable with 0ms LLM latency)
- 80% of users complete profile creation
- Content authoring time increase < 50% (acceptable tradeoff)
